const std = @import("std");


const MEM_SIZE: u16 = 0x1000;

const SCREEN_WIDTH: u8 = 64;
const SCREEN_HEIGHT: u8 = 32;


const C8Error = error {
StackPointer,
OutOfBound,
UnImplemented,
};

const Components = struct {
c: u8,
x: u8,
y: u8,
d: u8,
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
keypad: u16 = 0,
fb: [256]u8 = [_]u8 {0} ** 256,
tone: bool = false,
time: isize = 0,

  fn read_opcode(self: *Chip8) u16 {
    var op_byte1 = @intCast(u16,self.mem[self.pc]);
    var op_byte2 = @intCast(u16,self.mem[self.pc + 1]);
    return op_byte1 << 8 | op_byte2;

  }
  pub fn ret(self: *Chip8) C8Error!void {
    if (self.sp == 0) {
     return C8Error.StackPointer;
    }
    self.sp -= 1;
    self.pc = self.stack[self.sp];
  }

  fn add(self: *Chip8, x: u8, y: u8) C8Error!void {
    var arg1 = self.v[x];
    var arg2 = self.v[y];
    self.v[x] = arg1 + arg2;
  }

  fn call(self: *Chip8, addr: u16) C8Error!void {
    if (self.sp > self.stack.len) {
      return C8Error.OutOfBound;
    }
    self.stack[self.sp] = self.pc;
    self.sp +=1;
    self.pc = addr;
  }
  fn run(self: *Chip8) C8Error!void {
    while (true){
    var opcode = self.read_opcode();
    self.pc +=2;
    var comps = Components {
    .c = @intCast(u8,((opcode & 0xF000) >> 12)),
    .x = @intCast(u8,(opcode & 0x0F00) >> 8),
    .y = @intCast(u8,(opcode & 0x00F0) >> 4),
    .d = @intCast(u8,(opcode & 0x000F) >> 0),
    };
    var nnn = opcode & 0xFFF;
    if (comps.c == 0 and comps.x == 0 and comps.y == 0 and comps.d == 0) {
      //std.debug.print("DONE..\n", .{});
        return;
    }

    if (comps.c == 0 and comps.x == 0 and comps.y == 0xE and comps.d == 0xE) {

      //std.debug.print("return..\n", .{});
        try self.ret();
        continue;
    }
    if (comps.c == 0x2 ) {
      //std.debug.print("NNNN..\n", .{});
        try self.call(nnn);
    }
    if (comps.c == 0x8 and comps.d == 0x4) {
      //std.debug.print("ADDING..\n", .{});
      try self.add(comps.x,comps.y);
      continue;
    }
  }
  }
};

pub fn main() anyerror!void {
 comptime var cpu = Chip8{
  };
 cpu.v[0] = 5;
 cpu.v[1] = 10;
 
 cpu.mem[0x000] = 0x21; 
 cpu.mem[0x001] = 0x00; 
 
 cpu.mem[0x002] = 0x21; 
 cpu.mem[0x003] = 0x00; 
 
 cpu.mem[0x004] = 0x21; 
 cpu.mem[0x005] = 0x00; 
 
 cpu.mem[0x006] = 0x00; 
 cpu.mem[0x007] = 0x00; 
 
 cpu.mem[0x100] = 0x80; 
 cpu.mem[0x101] = 0x14; 
 cpu.mem[0x102] = 0x80; 
 cpu.mem[0x103] = 0x14; 
 cpu.mem[0x104] = 0x00; 
 cpu.mem[0x105] = 0xEE;

 //std.debug.print("{any}",.{cpu.mem});
 try comptime cpu.run();
 std.debug.print("{}",.{cpu.v[0]});



}

test "basic test" {
    try std.testing.expectEqual(10, 3 + 7);
}
