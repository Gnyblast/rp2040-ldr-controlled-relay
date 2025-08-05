const std = @import("std");
const microzig = @import("microzig");
const cron = @import("cron").Cron;
const datetime = @import("cron").datetime;
const time = rp2xxx.time;
const mz_time = microzig.drivers.time;
const rp2xxx = microzig.hal;
const gpio = rp2xxx.gpio;
const Pio = rp2xxx.pio.Pio;
const i2c = rp2xxx.i2c;
const StateMachine = rp2xxx.pio.StateMachine;
const font8x8 = @import("font8x8");

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
    .GPIO8 = .{
        .name = "SDA",
        .function = .I2C0_SDA,
        .schmitt_trigger = .enabled,
        .slew_rate = .slow,
        .pull = .up,
        .direction = .out,
    },
    .GPIO9 = .{
        .name = "SCL",
        .function = .I2C0_SCL,
        .schmitt_trigger = .enabled,
        .slew_rate = .slow,
        .pull = .up,
        .direction = .out,
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

const i2c0 = i2c.instance.num(0);
const lcd_address = rp2xxx.i2c.Address.new(0x3C);

pub fn main() void {
    pin_config.apply();
    rp2xxx.adc.apply(.{});
    rp2xxx.i2c.I2C.apply(i2c0, .{ .baud_rate = 400_000, .clock_config = rp2xxx.clock_config });
    rp2xxx.rtc.apply(rp2xxx.clock_config);

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

    const I2C_DEVICE = rp2xxx.drivers.I2C_Device.init(i2c0, lcd_address, null);
    const lcd = microzig.drivers.display.ssd1306.init(.i2c, I2C_DEVICE, null) catch unreachable;
    lcd.clear_screen(false) catch unreachable;
    const print_val = "Hello World!";
    var buff: [96]u8 = undefined;
    lcd.write_gdram(font8x8.Fonts.draw(&buff, print_val)) catch {
        pins.relay.put(OPEN);
    };

    var c = cron.init();
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

        c.parse("21 15 * * *") catch |err| {
            const error_name = @errorName(err);
            var new_buff: [100]u8 = undefined;
            lcd.write_gdram(font8x8.Fonts.draw(&new_buff, error_name)) catch {
                pins.relay.put(OPEN);
            };
            pins.relay.put(OPEN);
        };

        //const a: rp2xxx.i2c.Address = rp2xxx.i2c.Address.new(0x68);
        //const reg: [1]u8 = .{0x00};
        //_ = i2c0.write_blocking(a, &reg, mz_time.Duration.from_ms(100)) catch continue;

        //var rx_data: [7]u8 = undefined;
        //_ = i2c0.read_blocking(a, &rx_data, mz_time.Duration.from_ms(100)) catch continue;

        //const second = bcdToDec(rx_data[0]);
        //const minute = bcdToDec(rx_data[1]);
        //const hour = bcdToDec(rx_data[2] & 0b0011_1111); // Mask out 12/24h bits
        //const day = bcdToDec(rx_data[4]);
        //const month = bcdToDec(rx_data[5] & 0x1F); // Mask out century bit
        //const year = bcdToDec(rx_data[6]);

        //const now = datetime.Datetime.create(year, month, day, hour, minute, second, 0, datetime.timezones.Asia.Nicosia) catch continue;
        //const next = c.next(now) catch continue;

        //if (next.sub(now).seconds < 59) {
        //    pins.relay.put(OPEN);
        //}
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

fn bcdToDec(v: u8) u8 {
    return ((v >> 4) * 10) | (v & 0x0F);
}
