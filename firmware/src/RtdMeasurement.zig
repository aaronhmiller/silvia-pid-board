/// Simple RTD measurement using voltage divider and RP2040 ADC
/// WARNING: Much less accurate than MAX31865!
/// Expected accuracy: ±5-10°C
/// 
/// Circuit:
///   3.3V ─── R_ref (470Ω) ─┬─── ADC0 (GP26)
///                          │
///                        [RTD]
///                          │
///   GND ───────────────────┴────

const RtdMeasurement = @This();

const std = @import("std");
const microzig = @import("microzig");
const rp2040 = microzig.hal;
const gpio = rp2040.gpio;
const adc = rp2040.adc;
const time = rp2040.time;

const State = enum {
    idle,
    measuring,
};

state: State = .idle,
current_deadline_us: u64 = 0,
current_temperature: ?f32 = null,

const measurement_interval_ms = 100;

// Configuration - CHANGE THIS TO MATCH YOUR RESISTOR!
const R_REF: f32 = 470.0;          // Reference resistor in ohms (YOUR VALUE HERE!)
const R_RTD_0: f32 = 100.0;        // RTD resistance at 0°C (PT100)
const ADC_VREF: f32 = 3.3;         // ADC reference voltage
const ADC_MAX: f32 = 4096.0;       // 12-bit ADC (0-4095)

pub fn doWork(self: *RtdMeasurement, timestamp_us: u64) void {
    switch (self.state) {
        .idle => {
            if (timestamp_us >= self.current_deadline_us) {
                self.state = .measuring;
            }
        },
        .measuring => {
            // Perform blocking ADC conversion on channel 0 (GP26)
            const adc_value = adc.convert_one_shot_blocking(.ain0) catch {
                // Conversion error - skip this measurement
                self.current_temperature = null;
                self.current_deadline_us = timestamp_us + measurement_interval_ms * 1000;
                self.state = .idle;
                return;
            };
            
            // Convert ADC reading to voltage
            const v_adc: f32 = (@as(f32, @floatFromInt(adc_value)) / ADC_MAX) * ADC_VREF;
            
            // Calculate RTD resistance using voltage divider formula
            // V_adc = V_supply * R_rtd / (R_ref + R_rtd)
            // Solving for R_rtd:
            // R_rtd = (V_adc * R_ref) / (V_supply - V_adc)
            
            if (v_adc >= ADC_VREF - 0.01) {
                // Avoid division by zero or very small number
                self.current_temperature = null;
            } else {
                const r_rtd = (v_adc * R_REF) / (ADC_VREF - v_adc);
                
                // Convert resistance to temperature using simplified Callendar-Van Dusen
                self.current_temperature = resistanceToTemperature(r_rtd);
            }
            
            self.current_deadline_us = timestamp_us + measurement_interval_ms * 1000;
            self.state = .idle;
        },
    }
}

pub fn init(_: RtdMeasurement) void {
    // Initialize ADC hardware with default configuration
    adc.apply(.{});
    
    // Configure GP26 (ain0) for ADC use
    // This sets the pin function to .disabled, disables pulls, and disables digital input
    adc.configure_gpio_pin_num(.ain0);
    
    // Wait for ADC to stabilize
    time.sleep_ms(100);
}

fn resistanceToTemperature(resistance: f32) f32 {
    // Callendar-Van Dusen equation for PT100
    // Valid for temperatures above 0°C
    
    const rtd_a: f32 = 3.9083e-3;
    const rtd_b: f32 = -5.775e-7;
    
    const z1 = -rtd_a;
    const z2 = rtd_a * rtd_a - (4.0 * rtd_b);
    const z3 = (4.0 * rtd_b) / R_RTD_0;
    const z4 = 2.0 * rtd_b;
    
    const temp: f32 = (std.math.sqrt(z2 + (z3 * resistance)) + z1) / z4;
    
    // Basic sanity check
    if (temp < -50.0 or temp > 200.0) {
        // Temperature out of reasonable range, probably bad reading
        return 0.0;
    }
    
    return temp;
}

/// Alternative: Simple linear approximation (less accurate but simpler)
/// PT100: ~0.385Ω per °C
/// Uncomment this and use instead of resistanceToTemperature if you want simpler math
fn resistanceToTemperatureLinear(resistance: f32) f32 {
    const temp_coefficient: f32 = 0.00385; // Ω/Ω/°C for PT100
    return (resistance - R_RTD_0) / (R_RTD_0 * temp_coefficient);
}