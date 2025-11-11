/// Improved CLI for RP2040 Coffee Controller
/// Changes from original:
/// 1. Added command echo for debugging
/// 2. Structured response format with >> and << prefixes
/// 3. Better error messages
/// 4. Optional verbose mode
const Cli = @This();

const std = @import("std");
const main = @import("main.zig");
const heater_gpio = @import("HeaterControl.zig").heater_gpio;
const app_state: *main.AppState = &main.app_state;

// Configuration
const ENABLE_COMMAND_ECHO = true;  // Set to false to disable echo
const ENABLE_VERBOSE_RESPONSES = false;  // Set to true for more detailed responses

const ErrAndUsage = struct {
    err_msg: []const u8,
    usage: []const u8,
};

fn logHandler(argv: []const []const u8) ?ErrAndUsage {
    const usage =
        \\log on - Turn on periodic logging of temperature, setpoint and heater duty cycle
        \\log off - Turn off logging
    ;

    if (argv.len < 1) {
        main.send_response("<<ERROR: Expected 'on' or 'off'", .{});
        return .{ .err_msg = "Expected \"on\" or \"off\"", .usage = usage };
    }
    
    if (std.mem.eql(u8, argv[0], "on")) {
        app_state.temperature_logging = true;
        main.send_response("<<OK: Logging enabled", .{});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Temperature logging enabled", .{});
        }
    } else if (std.mem.eql(u8, argv[0], "off")) {
        app_state.temperature_logging = false;
        main.send_response("<<OK: Logging disabled", .{});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Temperature logging disabled", .{});
        }
    } else {
        main.send_response("<<ERROR: Expected 'on' or 'off', got '{s}'", .{argv[0]});
        return .{ .err_msg = "ERROR: expected \"on\" or \"off\"", .usage = usage };
    }

    return null;
}

fn tempHandler(_: []const []const u8) ?ErrAndUsage {
    if (app_state.rtd_measurement.current_temperature) |temp| {
        // Send machine-readable response with >> prefix
        main.send_response(">>TEMP,{d:.1}", .{temp});
        main.send_response("<<OK", .{});
        
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.debug("Temperature: {d:.1}°C", .{temp});
        }
    } else {
        main.send_response("<<ERROR: Temperature not ready", .{});
        const err = ErrAndUsage { 
            .err_msg = "ERROR: temp wait", 
            .usage = "needs input from temp sensor" 
        };
        return err;
    }

    return null;
}

fn statusHandler(_: []const []const u8) ?ErrAndUsage {
    // Output status in format expected by ESP32: STATUS,temp,setpoint,duty,state
    if (app_state.rtd_measurement.current_temperature) |temp| {
        const state_str = switch (app_state.temperature_regulation.state) {
            .off => "off",
            .init => "init",
            .thermostat_heating => "heating",
            .pid => "regulating",
        };
        
        // Send machine-readable response with >> prefix for data
        main.send_response(">>STATUS,{d:.1},{d:.1},{d},{s}", .{
            temp,
            app_state.temperature_regulation.setpoint,
            app_state.temperature_regulation.heater_control.duty_cycle,
            state_str,
        });
        main.send_response("<<OK", .{});
        
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.debug("Status: temp={d:.1} setpoint={d:.1} duty={d} state={s}", .{
                temp,
                app_state.temperature_regulation.setpoint,
                app_state.temperature_regulation.heater_control.duty_cycle,
                state_str,
            });
        }
    } else {
        main.send_response("<<ERROR: Temperature not ready", .{});
        const err = ErrAndUsage { 
            .err_msg = "ERROR: status wait", 
            .usage = "temp reading not ready" 
        };
        return err;
    }

    return null;
}

fn heaterHandler(argv: []const []const u8) ?ErrAndUsage {
    const usage =
        \\heater on - Turn on heater
        \\heater off - Turn off heater
        \\Note: this also stops temperature regulation loop
    ;

    if (argv.len < 1) {
        main.send_response("<<ERROR: Expected 'on' or 'off'", .{});
        return .{ .err_msg = "Expected \"on\" or \"off\"", .usage = usage };
    }
    
    if (std.mem.eql(u8, argv[0], "on")) {
        app_state.temperature_regulation.stop();
        heater_gpio.put(1);
        main.send_response("<<OK: Heater ON", .{});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Heater ON (manual)", .{});
        }
    } else if (std.mem.eql(u8, argv[0], "off")) {
        app_state.temperature_regulation.stop();
        heater_gpio.put(0);
        main.send_response("<<OK: Heater OFF", .{});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Heater OFF (manual)", .{});
        }
    } else {
        main.send_response("<<ERROR: Expected 'on' or 'off', got '{s}'", .{argv[0]});
        return .{ .err_msg = "ERROR: expected \"on\" or \"off\"", .usage = usage };
    }

    return null;
}

fn regHandler(argv: []const []const u8) ?ErrAndUsage {
    const usage =
        \\reg on - Turn on temperature regulation loop
        \\reg off - Turn off temperature regulation loop
        \\reg coffee [<temp>] - Change setpoint to coffee mode (default: 108°C)
        \\reg steam [<temp>] - Change setpoint to steam mode (default: 145°C)
    ;
    const err_msg = "ERROR: Expected \"on\", \"off\", \"coffee [temp]\", or \"steam [temp]\"";
    
    if (argv.len < 1) {
        main.send_response("<<ERROR: Missing argument", .{});
        return .{ .err_msg = err_msg, .usage = usage };
    }
    
    if (std.mem.eql(u8, argv[0], "on")) {
        app_state.temperature_regulation.start();
        main.send_response("<<OK: Regulation started", .{});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Starting temperature regulation loop", .{});
        }
    } else if (std.mem.eql(u8, argv[0], "off")) {
        app_state.temperature_regulation.stop();
        main.send_response("<<OK: Regulation stopped", .{});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Stopping temperature regulation loop", .{});
        }
    } else if (std.mem.eql(u8, argv[0], "coffee")) {
        const temperature: f32 = if (argv.len > 1)
            std.fmt.parseFloat(f32, argv[1]) catch {
                main.send_response("<<ERROR: Invalid temperature '{s}'", .{argv[1]});
                return .{ .err_msg = "Invalid temperature value", .usage = usage };
            }
        else
            108.0;
            
        if (temperature < 80.0 or temperature > 120.0) {
            main.send_response("<<ERROR: Temperature {d:.1}°C out of range (80-120°C)", .{temperature});
            return .{ .err_msg = "Temperature out of range", .usage = usage };
        }
        
        app_state.temperature_regulation.coffeeMode(temperature);
        main.send_response("<<OK: Coffee mode {d:.1}°C", .{temperature});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Changing setpoint to coffee mode: {d:.1}°C", .{temperature});
        }
    } else if (std.mem.eql(u8, argv[0], "steam")) {
        const temperature: f32 = if (argv.len > 1)
            std.fmt.parseFloat(f32, argv[1]) catch {
                main.send_response("<<ERROR: Invalid temperature '{s}'", .{argv[1]});
                return .{ .err_msg = "Invalid temperature value", .usage = usage };
            }
        else
            145.0;
            
        if (temperature < 120.0 or temperature > 160.0) {
            main.send_response("<<ERROR: Temperature {d:.1}°C out of range (120-160°C)", .{temperature});
            return .{ .err_msg = "Temperature out of range", .usage = usage };
        }
        
        app_state.temperature_regulation.steamMode(temperature);
        main.send_response("<<OK: Steam mode {d:.1}°C", .{temperature});
        if (ENABLE_VERBOSE_RESPONSES) {
            std.log.info("Changing setpoint to steam mode: {d:.1}°C", .{temperature});
        }
    } else {
        main.send_response("<<ERROR: Unknown subcommand '{s}'", .{argv[0]});
        return .{ .err_msg = err_msg, .usage = usage };
    }

    return null;
}

// NEW: Help command
fn helpHandler(_: []const []const u8) ?ErrAndUsage {
    main.send_response(">>Available commands:", .{});
    main.send_response(">>  status          - Get current temperature and state", .{});
    main.send_response(">>  temp            - Get current temperature only", .{});
    main.send_response(">>  log on|off      - Enable/disable periodic logging", .{});
    main.send_response(">>  heater on|off   - Manual heater control", .{});
    main.send_response(">>  reg on|off      - Start/stop temperature regulation", .{});
    main.send_response(">>  reg coffee [T]  - Coffee mode (80-120°C, default 108)", .{});
    main.send_response(">>  reg steam [T]   - Steam mode (120-160°C, default 145)", .{});
    main.send_response(">>  help            - Show this help", .{});
    main.send_response("<<OK", .{});
    return null;
}

const CommandHandlers = struct {
    const HandlerFnType = fn ([]const []const u8) ?ErrAndUsage;
    log: HandlerFnType = logHandler,
    temp: HandlerFnType = tempHandler,
    status: HandlerFnType = statusHandler,
    heater: HandlerFnType = heaterHandler,
    reg: HandlerFnType = regHandler,
    help: HandlerFnType = helpHandler,
};

const command_handlers: CommandHandlers = .{};

/// Parse and execute a command line string
/// Format: "command arg1 arg2 ..."
/// This is called directly from main loop for UART and USB CDC input
pub fn commandParser(cmd_line: []const u8) void {
    // Skip leading/trailing whitespace
    const trimmed = std.mem.trim(u8, cmd_line, " \r\n\t");
    
    if (trimmed.len == 0) return;
    
    // ADDED: Echo command for debugging (helps verify reception)
    if (ENABLE_COMMAND_ECHO) {
        main.send_response("<<CMD: {s}", .{trimmed});
    }
    
    // Find first space to separate command from args
    const space_idx = std.mem.indexOfScalar(u8, trimmed, ' ');
    const cmd_slice = if (space_idx) |idx| trimmed[0..idx] else trimmed;
    const args_start = if (space_idx) |idx| idx + 1 else trimmed.len;
    
    // Parse arguments (max 10 args supported)
    const max_num_args = 10;
    var args: [max_num_args][]const u8 = undefined;
    var arg_count: usize = 0;
    
    if (args_start < trimmed.len) {
        var iter = std.mem.tokenizeScalar(u8, trimmed[args_start..], ' ');
        while (iter.next()) |arg| {
            if (arg_count >= max_num_args) break;
            args[arg_count] = arg;
            arg_count += 1;
        }
    }
    
    // Find and execute command handler
    const ch_typeinfo = @typeInfo(CommandHandlers);
    inline for (ch_typeinfo.@"struct".fields) |field| {
        if (std.mem.eql(u8, cmd_slice, field.name)) {
            if (@field(command_handlers, field.name)(args[0..arg_count])) |err_and_usage| {
                if (ENABLE_VERBOSE_RESPONSES) {
                    std.log.err("Command: {s} produced Error: {s}", .{ cmd_slice, err_and_usage.err_msg });
                    std.log.err("Usage:\n{s}", .{err_and_usage.usage});
                }
            }
            return;
        }
    }
    
    // Unknown command
    main.send_response("<<ERROR: Unknown command '{s}' (try 'help')", .{cmd_slice});
    if (ENABLE_VERBOSE_RESPONSES) {
        std.log.err("Unknown command: {s}", .{cmd_slice});
    }
}

// Response format documentation:
// <<CMD: <command>     - Echo of received command (if ENABLE_COMMAND_ECHO = true)
// >>DATA               - Data line (e.g., STATUS, TEMP)
// <<OK[: message]      - Success completion marker
// <<ERROR: message     - Error completion marker