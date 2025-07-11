const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;
const Pio = rp2xxx.pio.Pio;
const StateMachine = rp2xxx.pio.StateMachine;

// Compile-time pin configuration
const pin_config = rp2xxx.pins.GlobalConfiguration{
    .GPIO29 = .{
        .name = "relay",
        .direction = .out,
    },
    .GPIO28 = .{
        .name = "sensor",
        .direction = .in,
    },
};

const ws2812_program = blk: {
    @setEvalBranchQuota(10_000);
    break :blk rp2xxx.pio.assemble(
        \\;
        \\; Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
        \\;
        \\; SPDX-License-Identifier: BSD-3-Clause
        \\;
        \\.program ws2812
        \\.side_set 1
        \\
        \\.define public T1 2
        \\.define public T2 5
        \\.define public T3 3
        \\
        \\.wrap_target
        \\bitloop:
        \\    out x, 1       side 0 [T3 - 1] ; Side-set still takes place when instruction stalls
        \\    jmp !x do_zero side 1 [T1 - 1] ; Branch on the bit we shifted out. Positive pulse
        \\do_one:
        \\    jmp  bitloop   side 1 [T2 - 1] ; Continue driving high, for a long pulse
        \\do_zero:
        \\    nop            side 0 [T2 - 1] ; Or drive low, for a short pulse
        \\.wrap
    , .{}).get_program_by_name("ws2812");
};

const pins = pin_config.pins();
const OPEN = 1;
const CLOSE = 0;
const pio: Pio = rp2xxx.pio.num(0);
const led_pin = gpio.num(16);
const sm: StateMachine = .sm0;

const RGB = extern struct {
    x: u8 = 0x00,
    b: u8,
    r: u8,
    g: u8,
};

const brightness: [256]u8 = blk: {
    @setEvalBranchQuota(25_000);

    const gamma = 2.2;

    const max_brightness = 0x10;

    var data: [256]u8 = undefined;
    for (&data, 0..) |*bit, i| {
        const raw_index: f32 = @floatFromInt(i);

        const gamma_brightness = std.math.pow(f32, raw_index / 255.0, gamma);

        bit.* = @intFromFloat(max_brightness * gamma_brightness);
    }
    // @compileLog(data);
    break :blk data;
};

pub fn main() void {
    pin_config.apply();
    rp2xxx.adc.apply(.{});
    pio.gpio_init(led_pin);
    sm_set_consecutive_pindirs(pio, sm, @intFromEnum(led_pin), 1, true);
    const cycles_per_bit: comptime_int = ws2812_program.defines[0].value + //T1
        ws2812_program.defines[1].value + //T2
        ws2812_program.defines[2].value; //T3
    const div = @as(f32, @floatFromInt(rp2xxx.clock_config.sys.?.frequency())) /
        (800_000 * cycles_per_bit);

    pio.sm_load_and_start_program(sm, ws2812_program, .{
        .clkdiv = rp2xxx.pio.ClkDivOptions.from_float(div),
        .pin_mappings = .{
            .side_set = .{
                .base = @intFromEnum(led_pin),
                .count = 1,
            },
        },
        .shift = .{
            .out_shiftdir = .left,
            .autopull = true,
            .pull_threshold = 24,
            .join_tx = true,
        },
    }) catch unreachable;
    pio.sm_set_enabled(sm, true);

    var screen: [5][5]RGB = undefined;
    for (&screen) |*row| {
        for (row) |*pix| {
            pix.* = RGB{
                .r = brightness[0],
                .g = brightness[255],
                .b = brightness[0],
            };
        }
    }

    for (@as([25]RGB, @bitCast(screen))) |color| {
        pio.sm_blocking_write(sm, @bitCast(color));
    }

    while (true) : (time.sleep_ms(1000)) {
        for (&screen) |*row| {
            for (row) |*pix| {
                pix.* = RGB{
                    .r = brightness[0],
                    .g = brightness[0],
                    .b = brightness[255],
                };
            }
        }

        for (@as([25]RGB, @bitCast(screen))) |color| {
            pio.sm_blocking_write(sm, @bitCast(color));
        }
        const val = rp2xxx.adc.convert_one_shot_blocking(.ain2) catch {
            continue;
        };
        if (val < 2000) {
            pins.relay.put(OPEN);
        } else {
            pins.relay.put(CLOSE);
        }
    }

    for (&screen) |*row| {
        for (row) |*pix| {
            pix.* = RGB{
                .r = brightness[255],
                .g = brightness[0],
                .b = brightness[0],
            };
        }
    }

    for (@as([25]RGB, @bitCast(screen))) |color| {
        pio.sm_blocking_write(sm, @bitCast(color));
    }
}

fn sm_set_consecutive_pindirs(_pio: Pio, _sm: StateMachine, pin: u5, count: u3, is_out: bool) void {
    const sm_regs = _pio.get_sm_regs(_sm);
    const pinctrl_saved = sm_regs.pinctrl.raw;
    sm_regs.pinctrl.modify(.{
        .SET_BASE = pin,
        .SET_COUNT = count,
    });
    _pio.sm_exec(_sm, rp2xxx.pio.Instruction{
        .tag = .set,
        .delay_side_set = 0,
        .payload = .{
            .set = .{
                .data = @intFromBool(is_out),
                .destination = .pindirs,
            },
        },
    });
    sm_regs.pinctrl.raw = pinctrl_saved;
}
