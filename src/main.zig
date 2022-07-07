const std = @import("std");
const c = @cImport({
    @cInclude("SDL2/SDL.h");
});

const expectEqual = std.testing.expectEqual;
//const sdl = @import("sdl2");
const MEM_SIZE: u16 = 0x1000;

const SCREEN_WIDTH: u16 = 64;
const SCREEN_HEIGHT: u16 = 32;
const FONTSET_START_ADDRESS: u16 = 0x50;

const FPS = 1000;
const SDL_KEYS = [_]c.SDL_Keycode {
   c.SDLK_x,c.SDLK_1,c.SDLK_2,c.SDLK_3,   // 0 1 2 3
   c.SDLK_q,c.SDLK_w,c.SDLK_e,c.SDLK_a,   // 4 5 6 7
   c.SDLK_s,c.SDLK_d,c.SDLK_z,c.SDLK_c,   // 8 9 A B
   c.SDLK_4,c.SDLK_r,c.SDLK_f,c.SDLK_v,  // C D E F
  };

const FONTS = [_]u8{
    0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
    0x20, 0x60, 0x20, 0x20, 0x70, // 1
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
    0x90, 0x90, 0xF0, 0x10, 0x10, // 4
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
    0xF0, 0x10, 0x20, 0x40, 0x40, // 7
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
    0xF0, 0x90, 0xF0, 0x90, 0x90, // A
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
    0xF0, 0x80, 0x80, 0x80, 0xF0, // C
    0xE0, 0x90, 0x90, 0x90, 0xE0, // D
    0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
    0xF0, 0x80, 0xF0, 0x80, 0x80, // F
};

const C8Error = error{
    StackPointer,
    OutOfBound,
    UnImplemented,
};
const Components = struct {
    c: u8,
    x: u8,
    y: u8,
    d: u8,
    nn: u8,
    nnn: u16,
};

const Chip8 = struct {
    mem: [MEM_SIZE]u8 = [_]u8{0} ** MEM_SIZE,
    v: [16]u8 = [_]u8{0} ** 16,
    i: u16 = 0,
    pc: u16 = 0,
    stack: [16]u16 = [_]u16{0} ** 16,
    sp: u8 = 0,
    dt: u8 = 0,
    st: u8 = 0,
    keypad: [16]u8 = [_]u8{0} ** 16,
    fb: [SCREEN_HEIGHT * SCREEN_WIDTH]u8 = [_]u8{0} ** (SCREEN_HEIGHT * SCREEN_WIDTH),
    tone: bool = false,
    time: isize = 0,
    drawfg: bool = false,

    // Utility Function
    fn read_opcode(self: *Chip8) u16 {
        var op_byte1 = @intCast(u16, self.mem[self.pc]);
        var op_byte2 = @intCast(u16, self.mem[self.pc + 1]);
        return op_byte1 << 8 | op_byte2;
    }
    // Clear Screen
    fn cls(self: *Chip8) void {
        comptime {
            self.fb = [_]u8{0} ** (SCREEN_WIDTH * SCREEN_HEIGHT);
        }
    }
    // return from subroutine
    fn ret(self: *Chip8) C8Error!void {
        if (self.sp == 0) {
            return C8Error.StackPointer;
        }
        self.sp -= 1;
        self.pc = self.stack[self.sp];
    }

    // 1nnn jump to addr nnn
    // Takes NNN as argument.
    fn jp(self: *Chip8, addr: u16) void {
        self.pc = addr;
    }

    // 8xy4
    // Add two registries together, stores result in vx
    fn addxy(self: *Chip8, x: u8, y: u8) void {
        var sum: u16 = self.v[x];
        sum += self.v[y];
        if (sum > 255) {
            self.v[0xF] = 1;
        } else {
            self.v[0xF] = 0;
        }
        self.v[x] = @intCast(u8, sum & 0xFF);
    }

    // skip if no keypress
    fn sknp(self: *Chip8, x: u8) void {
        var key = self.v[x];
        if (self.keypad[key] != 1) {
            self.pc += 2;
        }
    }

    // call function by incrementing the stack pointer and pc to the top of the stack.
    fn call(self: *Chip8, addr: u16) C8Error!void {
        if (self.sp > self.stack.len) {
            return C8Error.OutOfBound;
        }
        self.stack[self.sp] = self.pc;
        self.sp += 1;
        self.pc = addr;
    }
    //3xkk
    fn sex(self: *Chip8, x: u8, kk: u8) void {
        if (self.v[x] == kk) {
            self.pc += 2;
        }
    }

    //4xkk
    fn snex(self: *Chip8, x: u8, kk: u8) void {
        if (self.v[x] != kk) {
            self.pc += 2;
        }
    }
    //5xy0
    fn sexy(self: *Chip8, x: u8, y: u8) void {
        if (self.v[x] == self.v[y]) {
            self.pc += 2;
        }
    }
    //6xkk
    fn ldx(self: *Chip8, x: u8, kk: u8) void {
        self.v[x] = kk;
    }
    //7xkk
    // note, this has no carry.
    fn addx(self: *Chip8, x: u8, kk: u8) void {
        self.v[x] +%= kk;
    }

    // 8xy0
    fn ldxy(self: *Chip8, x: u8, y: u8) void {
        self.v[x] = self.v[y];
    }

    // 8xy1
    fn orxy(self: *Chip8, x: u8, y: u8) void {
        self.v[x] |= self.v[y];
    }

    // 8xy2
    fn andxy(self: *Chip8, x: u8, y: u8) void {
        self.v[x] &= self.v[y];
    }

    fn xorxy(self: *Chip8, x: u8, y: u8) void {
        self.v[x] ^= self.v[y];
    }

    // 8xy5
    fn subxy(self: *Chip8, x: u8, y: u8) void {
        if (self.v[x] > self.v[y]) {
            self.v[0xF] = 1;
        } else {
            self.v[0xF] = 0;
        }
        self.v[x] -%= self.v[y];
    }

    //8xy6
    fn shrx(self: *Chip8, x: u8) void {
        if (self.v[x] & 1 == 1) {
            self.v[0xF] = 1;
        } else {
            self.v[0xF] = 0;
        }
        self.v[x] >>= 1;
    }
    //8xy7
    fn subnxy(self: *Chip8, x: u8, y: u8) void {
        if (self.v[y] > self.v[x]) {
            self.v[0xF] = 1;
        } else {
            self.v[0xF] = 0;
        }
        self.v[x] = self.v[y] -% self.v[x];
    }
    //8xyE
    fn shlx(self: *Chip8, x: u8) void {
        if (self.v[x] & 1 == 1) {
            self.v[0xF] = 1;
        } else {
            self.v[0xF] = 0;
        }
        self.v[x] <<= 1;
    }

    //9xy0
    fn snexy(self: *Chip8, x: u8, y: u8) void {
        if (self.v[x] != self.v[y]) {
            self.pc += 2;
        }
    }
    //Annn
    fn ldi(self: *Chip8, nnn: u16) void {
        self.i = nnn;
    }

    //Bnnn
    fn jpv0(self: *Chip8, nnn: u16) void {
        self.pc = nnn + self.v[0];
    }

    //Cxkk
    fn rndx(self: *Chip8, x: u8, kk: u8) void {
        var seed = std.time.timestamp();
        if (seed < 0) {
            seed = 42;
        }
        var rand = std.rand.DefaultPrng.init(@intCast(u64, seed));
        const num = rand.random().int(u8);
        self.v[x] = num & kk;
    }
    //dxyn
    fn drwxyn(self: *Chip8, x: u8, y: u8, nibble: u8) void {
        var xpos = self.v[x] % SCREEN_WIDTH;
        var ypos = self.v[y] % SCREEN_HEIGHT;
        self.v[0xF] = 0;
        var row: u8 = 0;

        while (row < nibble) : (row += 1) {
            var col: u8 = 0;
            var sprite_byte: u8 = self.mem[self.i + row];
            while (col < 8) : (col += 1) {
                // Had to name it that way because this annoyed the shit out of me
                var shifted_ballsack: u8 = std.math.shr(u8, 0x80, col);
                var sprite_pixel: u8 = sprite_byte & shifted_ballsack;
                var screen_pixel = &self.fb[(((ypos + row) * SCREEN_WIDTH) + (xpos + col)) % (SCREEN_WIDTH * SCREEN_HEIGHT)];
                if (sprite_pixel != 0) {
                    if (screen_pixel.* == 1) {
                        std.log.info("COLLISION!!\n",.{});
                        self.v[0xF] = 1;
                    }
                    screen_pixel.* ^= 1;
                }

            }
        }
    }
    //ex9e
    fn skpx(self: *Chip8, x: u8) void {
        if (self.keypad[x] == 1) {
            self.pc += 2;
        }
    }

    //exA1
    fn sknpx(self: *Chip8, x: u8) void {
        if (self.keypad[x] == 0) {
            self.pc += 2;
        }
    }

    //fx07
    fn ldxdt(self: *Chip8, x: u8) void {
        self.v[x] = self.dt;
    }

    //fx0A
    fn ldxk(self: *Chip8, x: u8) void {
        var k: u8 = 0;
        while (k <= 15) : (k += 1) {
            if (self.keypad[k] == 1) {
                self.v[x] = k;
                return;
            }
        }
        self.pc -= 2;
    }

    //fx15
    fn lddtx(self: *Chip8, x: u8) void {
        self.dt = self.v[x];
    }

    //fx18
    fn ldstx(self: *Chip8, x: u8) void {
        self.st = self.v[x];
    }

    //fx1e
    fn addix(self: *Chip8, x: u8) void {
        self.i += self.v[x];
    }

    //fx29
    fn ldfx(self: *Chip8, x: u8) void {
        self.i = FONTSET_START_ADDRESS + (5 * self.v[x]);
    }
    //fx33
    fn ldbx(self: *Chip8, x: u8) void {
        var val = self.v[x];
        self.mem[self.i + 2] = self.v[x] % 10;
        val /= 10;

        self.mem[self.i + 1] = self.v[x] % 10;
        val /= 10;

        self.mem[self.i] = self.v[x] % 10;
    }

    //fx55
    fn ldix(self: *Chip8, x: u8) void {
        for (self.mem[self.i .. x + 1]) |*val, idx| {
            val.* = self.v[idx];
        }
    }

    //fx65
    fn ldxi(self: *Chip8, x: u8) void {
        for (self.v[0 .. x + 1]) |*val, idx| {
            val.* = self.mem[idx + self.i];
        }
    }

    fn loadRom(self: *Chip8, data: [*]const u8, len: u64) void {
        @memcpy(self.mem[0x200..], data, len);
        self.pc = 0x200;
    }

    fn run(self: *Chip8, quit_flag: *bool) C8Error!void {
        var opcode = self.read_opcode();
        self.pc += 2;
        var comps = Components{
            .c = @intCast(u8, ((opcode & 0xF000) >> 12)),
            .x = @intCast(u8, (opcode & 0x0F00) >> 8),
            .y = @intCast(u8, (opcode & 0x00F0) >> 4),
            .d = @intCast(u8, (opcode & 0x000F) >> 0),
            .nn = @intCast(u8, opcode & 0xFF),
            .nnn = opcode & 0xFFF,
        };
        switch (comps.c) {
          0x0 => {
            if (comps.x == 0x0 and comps.y == 0xE and comps.d == 0x0) {
              self.cls();
            } else if (comps.x == 0x0 and comps.y == 0xE and comps.d == 0xE) {
             try self.ret(); 
            } else if (comps.x == 0x0 and comps.y == 0x0 and comps.d == 0x0) {
              quit_flag.* = true;
            }
          },
          0x1 => self.jp(comps.nnn),
          0x2 => try self.call(comps.nnn),
          0x3 => self.sex(comps.x,comps.nn),
          0x4 => self.snex(comps.x, comps.nn),
          0x5 => {
            if (comps.d == 0) {
              self.sexy(comps.x, comps.y);
            }
          },
          0x6 => self.ldx(comps.x,comps.nn),
          0x7 => self.addx(comps.x,comps.nn),
          0x8 => {
              switch (comps.d) {
                0x0 => self.ldxy(comps.x,comps.y),
                0x1 => self.orxy(comps.x, comps.y),
                0x2 => self.andxy(comps.x, comps.y),
                0x3 => self.xorxy(comps.x, comps.y),
                0x4 => self.addxy(comps.x,comps.y),
                0x5 => self.subxy(comps.x, comps.y),
                0x6 => self.shrx(comps.x),
                0x7 => self.subnxy(comps.x, comps.y),
                0xE => self.shlx(comps.x),
               else => return C8Error.UnImplemented,
              }
          },
         0x9 => {
            if (comps.d == 0) {
              self.snexy(comps.x, comps.y);
            }
         },
         0xA => self.ldi(comps.nnn),
         0xB => self.jpv0(comps.nnn),
         0xC => self.rndx(comps.x, comps.nn),
         0xD => {
            self.drawfg = true;
            std.log.info("DRW..\n", .{});
            self.drwxyn(comps.x, comps.y, comps.d);
         },
         0xE => {
          if (comps.y == 0x9 and comps.d == 0xE) {
            self.skpx(comps.x);

          } else if (comps.y == 0xA and comps.d == 0x1) {
            self.sknp(comps.x);
          }
          else {
            return C8Error.UnImplemented;
          }
         },
         
         0xF => {
          if (comps.y == 0x0 and comps.d == 0x7) {
            self.ldxdt(comps.x);
          } else if (comps.y == 0x0 and comps.d == 0xA) {
            self.ldxk(comps.x);
          } else if (comps.y == 0x1 and comps.d == 0x5) {
            self.lddtx(comps.x);
          } else if (comps.y == 0x1 and comps.d == 0x8) {
            self.ldstx(comps.x);
          } else if (comps.y == 0x1 and comps.d == 0xE) {
            self.addix(comps.x);
          } else if (comps.y == 0x2 and comps.d == 0x9) {
            self.ldfx(comps.x);
          } else if (comps.y == 0x3 and comps.d == 0x3) {
            self.ldbx(comps.x);
          } else if (comps.y == 0x5 and comps.d == 0x5) {
            self.ldix(comps.x);
          } else if (comps.y == 0x6 and comps.d == 0x5) {
            self.ldxi(comps.x);
          } else {
            return C8Error.UnImplemented;
          }
         },

         else => return C8Error.UnImplemented
        }

        if (self.dt > 0) {
            self.dt -= 1;
        }
        if (self.st > 0) {
            self.st -= 1;
        }
    }
};

pub fn main() anyerror!void {

    //var screen = try sdl_utils.Render.render_init("CHIP 8");
    //defer screen.drop();

    // BEGIN SCREEN

    if (c.SDL_Init(c.SDL_INIT_VIDEO) != 0) {
        c.SDL_Log("Unable to initialize SDL: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    }
    defer c.SDL_Quit();
    const screen = c.SDL_CreateWindow("CHIP-8", c.SDL_WINDOWPOS_CENTERED, c.SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH * 10, SCREEN_HEIGHT * 10, c.SDL_WINDOW_OPENGL) orelse {
        c.SDL_Log("Unable to create window: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    };
    defer c.SDL_DestroyWindow(screen);

    const renderer = c.SDL_CreateRenderer(screen, -1, 0) orelse {
        c.SDL_Log("Unable to create renderer: %s", c.SDL_GetError());
        return error.SDLInitializationFailed;
    };
    defer c.SDL_DestroyRenderer(renderer);
    const texture = c.SDL_CreateTexture(renderer, c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, SCREEN_WIDTH, SCREEN_HEIGHT);
    defer c.SDL_DestroyTexture(texture);

    // BEGIN CPU
    var cpu = Chip8{};

    comptime @memcpy(cpu.mem[FONTSET_START_ADDRESS..], &FONTS, FONTS.len);
    if (std.os.argv.len > 1) {
        std.log.info("loading rom from {s}", .{std.os.argv[1]});
        const file = try std.fs.cwd().openFile(
            std.mem.span(std.os.argv[1]),
            .{ .mode = std.fs.File.OpenMode.read_only },
        );
        defer file.close();
        var data: [4096]u8 = undefined;
        const n = try file.readAll(&data);
        cpu.loadRom(&data,n);
        //load fonts
    } else {
        std.log.info("no rom specified", .{});
    }

 var flag = false;
    var clock: f32 = 0;
    screenloop: while (!flag) {
        var event: c.SDL_Event = undefined;
        while (c.SDL_PollEvent(&event) != 0) {
            switch (event.type) {
                c.SDL_QUIT => break :screenloop,
                c.SDL_KEYDOWN => {
                  for (SDL_KEYS) |key,idx| {
                    if (event.key.keysym.sym == key) {
                       cpu.keypad[idx] = 1;
                    }

                  }
                },
                c.SDL_KEYUP => {
                  for (SDL_KEYS) |key,idx| {
                    if (event.key.keysym.sym == key) {
                       cpu.keypad[idx] = 0;
                    }
                  }
                },
                else => {},
            }
        }
        try cpu.run(&flag);
        var new_clock: f32 = @intToFloat(f32, c.SDL_GetTicks());
        var delta_ticks = 1000 / FPS - (new_clock - clock);
        if (delta_ticks > 0) {
            _ = c.SDL_Delay(@floatToInt(u32, delta_ticks));
        }
        if (delta_ticks < -30) {
            clock = new_clock - 30;
        } else {
            clock = new_clock + delta_ticks;
        }
        //start drawing
        if (cpu.drawfg) {
          cpu.drawfg = false;
        var tt = [_]u32{0} ** (SCREEN_HEIGHT * SCREEN_WIDTH);
        drawMe(&cpu.fb, &tt);
        _ = c.SDL_UpdateTexture(texture, null, @ptrCast(*const anyopaque, &tt), SCREEN_WIDTH * @sizeOf(u32));
        _ = c.SDL_RenderCopy(renderer, texture, null, null);
        _ = c.SDL_RenderPresent(renderer);

        //std.log.info("{any}\n", .{tt});
        }
    }
}

// draw me
fn drawMe(cpu_fb: *[SCREEN_WIDTH * SCREEN_HEIGHT]u8, target: *[SCREEN_WIDTH * SCREEN_HEIGHT]u32) void {
    for (target) |*val, idx| {
        if (cpu_fb[idx] == 0) {
            val.* = 0x000000FF;
        } else {
            val.* = 0xFFFFFFFF;
        }
    }
}

//TODO 2: Test All Funcs

test "sub x y test (8xy5)" {
    var cpu = Chip8{};
    cpu.v[0] = 0x2D;
    cpu.v[1] = 0x4B;
    cpu.v[0xF] = 1;
    cpu.subxy(0, 1);
    try expectEqual(@as(u8, 0xE2), cpu.v[0]);
    try expectEqual(@as(u8, 0), cpu.v[0xF]);
}

test "8xy4 test (addxy)" {
    var cpu = Chip8{};

    // Carry
    cpu.v[0] = 0xED;
    cpu.v[1] = 0x4B;
    cpu.addxy(0, 1);
    try expectEqual(@as(u8, 0x38), cpu.v[0]);
    try expectEqual(@as(u8, 1), cpu.v[0xF]);
    // No Carry
    cpu.v[0] = 0x2D;
    cpu.v[1] = 0x4B;
    cpu.addxy(0, 1);
    try expectEqual(@as(u8, 0x78), cpu.v[0]);
    try expectEqual(@as(u8, 0), cpu.v[0xF]);
}

test "shlx test" {
    var cpu = Chip8{};
    cpu.v[0] = 5;
    cpu.shlx(0);
    try expectEqual(@as(u8, 10), cpu.v[0]);
    try expectEqual(@as(u8, 1), cpu.v[0xF]);
}

test "basic test" {
    try expectEqual(10, 3 + 7);
}

test "fx55 correctness test" {
    var cpu = Chip8{};
    cpu.v[0] = 5;
    cpu.v[1] = 6;
    cpu.v[2] = 7;
    cpu.v[3] = 8;
    // Deliberately not including the 8 to test the idx.
    cpu.ldix(2);
    try expectEqual(cpu.mem[0..5].*, [_]u8{ 5, 6, 7, 0, 0 });
}
