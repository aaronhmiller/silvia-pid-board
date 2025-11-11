const std = @import("std");
const microzig = @import("microzig");
const irq = @import("irq.zig");
const rp2040 = microzig.hal;
const time = rp2040.time;
const usb = rp2040.usb;
const HeaterControl = @import("HeaterControl.zig");
const TemperatureRegulation = @import("TemperatureRegulation.zig");
const RtdMeasurement = @import("RtdMeasurement.zig");
const DataLogger = @import("DataLogger.zig");
const Cli = @import("Cli.zig");

const gpio = rp2040.gpio;
const led = gpio.num(25);
const uart = rp2040.uart.instance.num(0);
const uart_tx_pin = gpio.num(0);
const uart_rx_pin = gpio.num(1);

const usb_dev = rp2040.usb.Usb(.{});

const usb_config_len = usb.templates.config_descriptor_len + usb.templates.cdc_descriptor_len;
const usb_config_descriptor =
    usb.templates.config_descriptor(1, 2, 0, usb_config_len, 0xc0, 100) ++
    usb.templates.cdc_descriptor(0, 4, usb.Endpoint.to_address(1, .In), 8, usb.Endpoint.to_address(2, .Out), usb.Endpoint.to_address(2, .In), 64);

var driver_cdc: usb.cdc.CdcClassDriver(usb_dev) = .{};
var drivers = [_]usb.types.UsbClassDriver{driver_cdc.driver()};

pub var DEVICE_CONFIGURATION: usb.DeviceConfiguration = .{
    .device_descriptor = &.{
        .descriptor_type = usb.DescType.Device,
        .bcd_usb = 0x0200,
        .device_class = 0xEF,
        .device_subclass = 2,
        .device_protocol = 1,
        .max_packet_size0 = 64,
        .vendor = 0x2E8A,
        .product = 0x000a,
        .bcd_device = 0x0100,
        .manufacturer_s = 1,
        .product_s = 2,
        .serial_s = 0,
        .num_configurations = 1,
    },
    .config_descriptor = &usb_config_descriptor,
    .lang_descriptor = "\x04\x03\x09\x04",
    .descriptor_strings = &.{
        &usb.utils.utf8_to_utf16_le("Raspberry Pi"),
        &usb.utils.utf8_to_utf16_le("Silvia Coffee Controller"),
        &usb.utils.utf8_to_utf16_le("silvia01"),
        &usb.utils.utf8_to_utf16_le("Coffee CDC"),
    },
    .drivers = &drivers,
};

// Configuration flags
const ENABLE_TEMPERATURE_SENSOR = true;  // Set to true when sensor is connected
const ENABLE_INTERRUPTS = false;          // Set to true when hardware is ready

// Flag to enable USB logging only after USB is initialized
var usb_logging_ready: bool = false;

// Simplified log function - no-op for now to avoid any issues
pub fn log(
    comptime level: std.log.Level,
    comptime scope: @TypeOf(.EnumLiteral),
    comptime format: []const u8,
    args: anytype,
) void {
    _ = level;
    _ = scope;
    _ = format;
    _ = args;
}

pub fn panic(_: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    // ALWAYS turn off heater
    HeaterControl.heater_gpio.put(0);

    // Flash LED rapidly to indicate panic
    led.set_direction(.out);
    led.set_function(.sio);
    while (true) {
        led.put(1);
        time.sleep_ms(100);
        led.put(0);
        time.sleep_ms(100);
    }
}

pub const microzig_options = microzig.Options{
    .log_level = .debug,
    .logFn = log,
    .interrupts = if (ENABLE_INTERRUPTS) .{
        .IO_IRQ_BANK0 = .{ .c = &irq.gpio },
        .HardFault = .{ .c = &irq.hardFault },
        .TIMER_IRQ_0 = .{ .c = &irq.alarmExpired },
    } else .{},
};

const Blink = struct {
    const onboard_led_gpio = rp2040.gpio.num(25);
    const blink_period_ms = 1000;
    blink_deadline: u64 = 0,

    pub fn init(self: *Blink, timestamp_us: u64) void {
        onboard_led_gpio.set_direction(.out);
        onboard_led_gpio.set_function(.sio);
        onboard_led_gpio.put(1);
        self.blink_deadline = timestamp_us + blink_period_ms * 1000;
    }

    pub fn doWork(self: *Blink, timestamp_us: u64) void {
        if (timestamp_us >= self.blink_deadline) {
            onboard_led_gpio.toggle();
            self.blink_deadline = timestamp_us + blink_period_ms * 1000;
        }
    }
};

var usb_tx_buff: [1024]u8 = undefined;

pub fn usb_cdc_write(comptime fmt: []const u8, args: anytype) void {
    const text = std.fmt.bufPrint(&usb_tx_buff, fmt, args) catch &.{};

    var remaining: []const u8 = text;
    while (remaining.len > 0) {
        remaining = driver_cdc.write(remaining);
        usb_dev.task(false) catch unreachable;
    }
    _ = driver_cdc.write_flush();
    usb_dev.task(false) catch unreachable;
}

var usb_rx_buff: [1024]u8 = undefined;

pub fn usb_cdc_read() []const u8 {
    var total_read: usize = 0;
    var read_buff: []u8 = usb_rx_buff[0..];

    while (true) {
        const len = driver_cdc.read(read_buff);
        read_buff = read_buff[len..];
        total_read += len;
        if (len == 0) break;
    }

    return usb_rx_buff[0..total_read];
}

// Structured response function for command results
var response_buff: [512]u8 = undefined;

pub fn send_response(comptime fmt: []const u8, args: anytype) void {
    const text = std.fmt.bufPrint(&response_buff, fmt ++ "\n", args) catch &.{};
    
    // Send via UART (to ESP32) - using direct register access
    const uart0_base = 0x40034000;
    const uartdr = @as(*volatile u32, @ptrFromInt(uart0_base + 0x000));
    const uartfr = @as(*volatile u32, @ptrFromInt(uart0_base + 0x018));
    const txff_mask: u32 = (1 << 5);
    
    for (text) |byte| {
        while ((uartfr.* & txff_mask) != 0) {}
        uartdr.* = byte;
    }
    
    // Also send via USB CDC if ready
    if (usb_logging_ready) {
        var remaining: []const u8 = text;
        while (remaining.len > 0) {
            remaining = driver_cdc.write(remaining);
            usb_dev.task(false) catch unreachable;
        }
        _ = driver_cdc.write_flush();
        usb_dev.task(false) catch unreachable;
    }
}

// Helper to display strings with escaped control characters
var display_buffer: [2048]u8 = undefined;

pub fn escapeForDisplay(input: []const u8) []const u8 {
    var out_idx: usize = 0;
    
    for (input) |byte| {
        if (out_idx >= display_buffer.len - 4) break;
        
        switch (byte) {
            '\n' => {
                display_buffer[out_idx] = '\\';
                display_buffer[out_idx + 1] = 'n';
                out_idx += 2;
            },
            '\r' => {
                display_buffer[out_idx] = '\\';
                display_buffer[out_idx + 1] = 'r';
                out_idx += 2;
            },
            '\t' => {
                display_buffer[out_idx] = '\\';
                display_buffer[out_idx + 1] = 't';
                out_idx += 2;
            },
            0...8, 11...12, 14...31, 127 => {
                display_buffer[out_idx] = '\\';
                display_buffer[out_idx + 1] = 'x';
                const hex = "0123456789ABCDEF";
                display_buffer[out_idx + 2] = hex[byte >> 4];
                display_buffer[out_idx + 3] = hex[byte & 0x0F];
                out_idx += 4;
            },
            else => {
                display_buffer[out_idx] = byte;
                out_idx += 1;
            },
        }
    }
    
    return display_buffer[0..out_idx];
}

// UART line buffer
var uart_line_buffer: [1024]u8 = undefined;
var uart_line_index: usize = 0;

pub fn uart_read_line() ?[]const u8 {
    const uart0_base = 0x40034000;
    const uartdr = @as(*volatile u32, @ptrFromInt(uart0_base + 0x000));
    const uartfr = @as(*volatile u32, @ptrFromInt(uart0_base + 0x018));
    const rxfe_mask: u32 = (1 << 4);
    
    while (uart_line_index < uart_line_buffer.len) {
        if ((uartfr.* & rxfe_mask) != 0) {
            break;
        }
        
        const byte = @as(u8, @truncate(uartdr.* & 0xFF));
        uart_line_buffer[uart_line_index] = byte;
        uart_line_index += 1;
        
        if (byte == '\n') {
            const line = uart_line_buffer[0..uart_line_index];
            uart_line_index = 0;
            return line;
        }
    }
    
    if (uart_line_index >= uart_line_buffer.len) {
        uart_line_index = 0;
    }
    
    return null;
}

pub const AppState = struct {
    blink: Blink = .{},
    data_logger: DataLogger = .{},
    temperature_regulation: TemperatureRegulation = .{},
    rtd_measurement: RtdMeasurement = .{},
    temperature_logging: bool = false,

    pub fn init(self: *AppState, timestamp_us: u64) void {
        // Initialize LED blink (always safe)
        self.blink.init(timestamp_us);
        
        // Initialize data logger (safe)
        self.data_logger.init(timestamp_us);
        
        // Initialize temperature regulation (safe)
        self.temperature_regulation.init();
        
        // Initialize RTD measurement ONLY if enabled
        if (ENABLE_TEMPERATURE_SENSOR) {
            self.rtd_measurement.init();
        }
    }
};

pub var app_state: AppState = .{};

pub fn main() !void {
    // Initialize LED first (for status indication)
    led.set_direction(.out);
    led.set_function(.sio);
    led.put(1);
    
    // Initialize interrupts if enabled
    if (ENABLE_INTERRUPTS) {
        irq.init();
    }

    var cached_steam_state: bool = false;

    // Initialize UART pins
    inline for (&.{ uart_tx_pin, uart_rx_pin }) |pin| {
        pin.set_function(.uart);
    }

    // Initialize UART
    uart.apply(.{
        .baud_rate = 115200,
        .clock_config = rp2040.clock_config,
    });

    // Initialize USB
    usb_dev.init_clk();
    usb_dev.init_device(&DEVICE_CONFIGURATION) catch unreachable;
    
    // CRITICAL: Poll USB task during enumeration window
    var enum_count: u32 = 0;
    while (enum_count < 200000) : (enum_count += 1) {
        usb_dev.task(false) catch {};
    }
    
    // USB is ready
    usb_logging_ready = true;
    led.put(0);  // LED off = ready
    
    // Send startup message
    send_response("=== Coffee Controller Started ===", .{});
    send_response("Temperature sensor: {s}", .{if (ENABLE_TEMPERATURE_SENSOR) "ENABLED" else "DISABLED"});
    send_response("Interrupts: {s}", .{if (ENABLE_INTERRUPTS) "ENABLED" else "DISABLED"});
    
    // Initialize app state AFTER USB is ready
    app_state.init(time.get_time_since_boot().to_us());
    
    // Command buffer
    var cmd_buffer: [1024]u8 = undefined;
    
    // USB CDC line buffer (like the test firmware)
    var usb_line_buffer: [1024]u8 = undefined;
    var usb_line_index: usize = 0;

    while (true) {
        // PRIORITY 1: USB task (must be called frequently)
        usb_dev.task(false) catch unreachable;

        // PRIORITY 2: Read and process USB CDC commands with line buffering
        var usb_byte_buffer: [1]u8 = undefined;
        const usb_len = driver_cdc.read(&usb_byte_buffer);
        
        if (usb_len > 0) {
            const byte = usb_byte_buffer[0];
            
            if (byte == '\n' or byte == '\r') {
                if (usb_line_index > 0) {
                    // Process complete command
                    const usb_cmd_len = @min(usb_line_index, cmd_buffer.len);
                    @memcpy(cmd_buffer[0..usb_cmd_len], usb_line_buffer[0..usb_cmd_len]);
                    Cli.commandParser(cmd_buffer[0..usb_cmd_len]);
                    usb_line_index = 0;
                }
            } else if (byte >= 32 and byte < 127) {
                if (usb_line_index < usb_line_buffer.len) {
                    usb_line_buffer[usb_line_index] = byte;
                    usb_line_index += 1;
                }
            }
        }
        
        // PRIORITY 3: Read and process UART commands
        if (uart_read_line()) |line| {
            const uart_cmd_len = @min(line.len, cmd_buffer.len);
            @memcpy(cmd_buffer[0..uart_cmd_len], line[0..uart_cmd_len]);
            Cli.commandParser(cmd_buffer[0..uart_cmd_len]);
        }
        
        // PRIORITY 4: Handle steam state changes (if interrupts enabled)
        if (ENABLE_INTERRUPTS) {
            if (cached_steam_state != irq.steam_enabled) {
                cached_steam_state = irq.steam_enabled;
                if (cached_steam_state) {
                    app_state.temperature_regulation.steamMode(145.0);
                } else {
                    app_state.temperature_regulation.coffeeMode(108.0);
                }
            }
        }
    	
        // PRIORITY 5: LED blink (visual status)
        app_state.blink.doWork(time.get_time_since_boot().to_us());
        
        // PRIORITY 6: Temperature measurement and regulation (if enabled)
        if (ENABLE_TEMPERATURE_SENSOR) {
            app_state.rtd_measurement.doWork(time.get_time_since_boot().to_us());
            if (app_state.rtd_measurement.current_temperature) |v| {
                try app_state.data_logger.doWork(
                    time.get_time_since_boot().to_us(),
                    v,
                    app_state.temperature_regulation.heater_control.duty_cycle,
                    app_state.temperature_regulation.setpoint,
                    app_state.temperature_logging,
                );
                app_state.temperature_regulation.doWork(time.get_time_since_boot().to_us(), v);
            }
        }
    }
}