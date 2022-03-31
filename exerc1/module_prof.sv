//////////////////////////////////////////////////////////////////////////////////
// Company: UFSCar
// Author: Ricardo Menotti
// 
// Create Date: 27.05.2021 13:15:28
// Project Name: Lab. Remoto de Lógica Digital - DC/UFSCar
// Design Name: VGA Test + Memory (32 bits)
// Module Name: vga
// Target Devices: xc7z020
// Tool Versions: Vivado v2019.2 (64-bit)
//////////////////////////////////////////////////////////////////////////////////

module top(
  input sysclk, // 125MHz
  output [3:0] led,
  output led5_r, led5_g, led5_b, led6_r, led6_g, led6_b,
  output [3:0] VGA_R, VGA_G, VGA_B, 
  output VGA_HS_O, VGA_VS_O);

  wire pixel_clk, reset, we; 
  wire [31:0] vdata;
  wire [ 6:0] vaddr; // 2^7 = 128
  
  power_on_reset por(sysclk, reset);
  clk_wiz_1 clockdiv(pixel_clk, sysclk); // 25MHz
  mem #("mario32.bin") ram(sysclk, we, address, data, vaddr, vdata); 
  vga video(pixel_clk, reset, vdata, vaddr, VGA_R, VGA_G, VGA_B, VGA_HS_O, VGA_VS_O);
endmodule

module vga( // 20x15 
  input clk, reset,
  input  [31:0] vdata,
  output [ 6:0] vaddr, 
  output [3:0] VGA_R, VGA_G, VGA_B, 
  output VGA_HS_O, VGA_VS_O);

  reg [9:0] CounterX, CounterY;
  reg inDisplayArea;
  reg vga_HS, vga_VS;

  wire CounterXmaxed = (CounterX == 800); // 16 + 48 + 96 + 640
  wire CounterYmaxed = (CounterY == 525); // 10 +  2 + 33 + 480
  wire [4:0] col;
  wire [3:0] row;
  wire [7:0] vbyte;

  always @(posedge clk or posedge reset)
    if (reset)
      CounterX <= 0;
    else 
      if (CounterXmaxed)
        CounterX <= 0;
      else
        CounterX <= CounterX + 1;

  always @(posedge clk or posedge reset)
    if (reset)
      CounterY <= 0;
    else 
      if (CounterXmaxed)
        if(CounterYmaxed)
          CounterY <= 0;
        else
          CounterY <= CounterY + 1;

  assign row = (CounterY>>5); // 32 pixels x
  assign col = (CounterX>>5); // 32 pixels (x4 bytes)
  assign vaddr = (col>>2) + (row<<2) + row; // addr = col / 4 + row * 5 
  assign vbyte = col[1] ? (col[0] ? vdata[7:0] : vdata[15:8]) : (col[0] ? vdata[23:16] : vdata[31:24]); // byte select 

  always @(posedge clk)
  begin
    vga_HS <= (CounterX > (640 + 16) && (CounterX < (640 + 16 + 96)));   // active for 96 clocks
    vga_VS <= (CounterY > (480 + 10) && (CounterY < (480 + 10 +  2)));   // active for  2 clocks
    inDisplayArea <= (CounterX < 640) && (CounterY < 480);
  end

  assign VGA_HS_O = ~vga_HS;
  assign VGA_VS_O = ~vga_VS;  

  assign VGA_R = inDisplayArea ? {vbyte[5:4], 2'b00} : 4'b0000;
  assign VGA_G = inDisplayArea ? {vbyte[3:2], 2'b00} : 4'b0000;
  assign VGA_B = inDisplayArea ? {vbyte[1:0], 2'b00} : 4'b0000;
endmodule


module mem #(parameter filename = "ram.hex")
          (input clock, we,
           input [6:0] address,
           inout [31:0] data,
           input [6:0] vaddr,
           output [31:0] vdata);

  logic [31:0] RAM[127:0];

  initial
    $readmemb(filename, RAM);

  assign data  = we ? 'bz : RAM[address]; 
  assign vdata = RAM[vaddr]; 

  always @(posedge clock)
    if (we) RAM[address] <= data;
endmodule

module power_on_reset(
  input clk, 
  output reset);

  reg q0 = 1'b0;
  reg q1 = 1'b0;
  reg q2 = 1'b0;
 
  always@(posedge clk)
  begin
       q0 <= 1'b1;
       q1 <= q0;
       q2 <= q1;
  end

  assign reset = !(q0 & q1 & q2);
endmodule


