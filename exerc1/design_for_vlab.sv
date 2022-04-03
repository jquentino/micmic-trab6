module top(input  logic        sysclk,
           output       [3:0]  VGA_R, VGA_G, VGA_B, 
           output              VGA_HS_O, VGA_VS_O);
           

  logic [31:0] WriteData, DataAdr, vdata, vaddr; 
  logic        MemWrite, pixel_clk, reset;


  logic [31:0] PC, Instr, ReadData;
  
  // instantiate processor and memories
  arm arm(pixel_clk, reset, PC, Instr, MemWrite, DataAdr, 
          WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(pixel_clk, MemWrite, DataAdr, WriteData, vaddr, ReadData, vdata);
  //Incluir vga aqui e arrumar portas de RGB, poweronreset, vga, clk_wiz_1
  
//===============================================================================================================
  power_on_reset por(pixel_clk, reset);
  clk_wiz_1 clockdiv(pixel_clk, sysclk); // 25MHz 
  vga video(pixel_clk, reset, vdata, vaddr, VGA_R, VGA_G, VGA_B, VGA_HS_O, VGA_VS_O);
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

// Substitutes Xilinx IP core for simulation purposes
module clk_wiz_1(output clk_out, input clk_in);
   assign clk_out = clk_in;
endmodule

module vga( // 20x15 
  input clk, reset,
  input  [31:0] vdata,
  output [6:0] vaddr, // 2^7 = 128
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
  assign col = (CounterX>>5); // 32 pixels 
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

  //assign VGA_R = inDisplayArea ? {vdata[5:4], 2'b00} : 4'b0000;
  //assign VGA_G = inDisplayArea ? {vdata[3:2], 2'b00} : 4'b0000;
  //assign VGA_B = inDisplayArea ? {vdata[1:0], 2'b00} : 4'b0000;
  
  assign VGA_R = inDisplayArea ? {vbyte[5:4], 2'b00} : 4'b0000;
  assign VGA_G = inDisplayArea ? {vbyte[3:2], 2'b00} : 4'b0000;
  assign VGA_B = inDisplayArea ? {vbyte[1:0], 2'b00} : 4'b0000;
endmodule

//===============================================================================================================

module dmem(input  logic        clk, MemWrite,
            input  logic [31:0] a, wd, va,
            output logic [31:0] rd, vd);

  logic [31:0] RAM[127:0];
  
  initial
    $readmemb("luigi32.bin", RAM);

  assign vd = RAM[va]; 
  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (MemWrite) RAM[a[31:2]] <= wd;
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("memfile.dat",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module arm(input  logic        clk, reset,
           output logic [31:0] PC,
           input  logic [31:0] Instr,
           output logic        MemWrite,
           output logic [31:0] ALUResult, WriteData,
           input  logic [31:0] ReadData);

  logic [3:0] ALUFlags;
  logic       RegWrite, 
              ALUSrc, MemtoReg, PCSrc, CS_RSB;
  logic [1:0] RegSrc, ImmSrc, ALUControl;

  controller c(clk, reset, Instr[31:12], ALUFlags, 
               RegSrc, RegWrite, ImmSrc, 
               ALUSrc, CS_RSB, ALUControl,
               MemWrite, MemtoReg, PCSrc);
  datapath dp(clk, reset, 
              RegSrc, RegWrite, ImmSrc,
              ALUSrc, CS_RSB, ALUControl,
              MemtoReg, PCSrc,
              ALUFlags, PC, Instr,
              ALUResult, WriteData, ReadData);
endmodule

module controller(input  logic         clk, reset,
	              input  logic [31:12] Instr,
                  input  logic [3:0]   ALUFlags,
                  output logic [1:0]   RegSrc,
                  output logic         RegWrite,
                  output logic [1:0]   ImmSrc,
                  output logic         ALUSrc, 
                  output logic         CS_RSB,
                  output logic [1:0]   ALUControl,
                  output logic         MemWrite, MemtoReg,
                  output logic         PCSrc);

  logic [1:0] FlagW;
  logic       PCS, RegW, MemW;
  logic       CS_TST_CMN;
  
  decode dec(Instr[27:26], Instr[25:20], Instr[15:12],
             FlagW, PCS, RegW, MemW,
             MemtoReg, ALUSrc, CS_RSB, ImmSrc, RegSrc, ALUControl, CS_TST_CMN);
  condlogic cl(clk, reset, Instr[31:28], ALUFlags,
               FlagW, PCS, RegW, MemW,
               PCSrc, RegWrite, MemWrite, CS_TST_CMN);
endmodule

module decode(input  logic [1:0] Op,
              input  logic [5:0] Funct,
              input  logic [3:0] Rd,
              output logic [1:0] FlagW,
              output logic       PCS, RegW, MemW,
              output logic       MemtoReg, ALUSrc, CS_RSB,
              output logic [1:0] ImmSrc, RegSrc, ALUControl,
              output logic       CS_TST_CMN);

  logic [9:0] controls;
  logic       Branch, ALUOp;

  // Main Decoder
  
  always_comb
  	casex(Op)
  	                        
  	  2'b00: if (Funct[5])  controls = 10'b0000101001; 		// Data processing immediate
  	                        
  	         else           controls = 10'b0000001001; 		// Data processing register
  	                        
  	  2'b01: if (Funct[0])  controls = 10'b0001111000;		// LDR 
  	                        
  	         else   
             begin
               if (Funct[2])
                   controls = 11'b10011101001;				// STR
               else
                    controls = 10'b1001110100;				//STRB <===== NOVO
             end
  	                       
  	  2'b10:                controls = 10'b0110100010; 		// B
  	                        
  	  default:              controls = 10'bx;     			// Unimplemented     
  	endcase

  assign {RegSrc, ImmSrc, ALUSrc, MemtoReg, 
          RegW, MemW, Branch, ALUOp} = controls; 
          
  // ALU Decoder             
  always_comb
    if (ALUOp) begin                 // which DP Instr?
      case(Funct[4:1]) 
  	   		 4'b0100: // ADD
		     begin
				CS_RSB = 1'b0;
                CS_TST_CMN = 1'b0;
		        ALUControl = 2'b00;
		     end
  	    4'b0010: // SUB
		     begin
				CS_RSB = 1'b0;
                CS_TST_CMN = 1'b0;
		        ALUControl = 2'b01;
		     end
  	    4'b0011: // RSB <===== NOVO
		     begin
				CS_RSB = 1'b1;
                CS_TST_CMN = 1'b0;
		        ALUControl = 2'b01;
		     end
            4'b0000: // AND
		     begin
			    CS_RSB = 1'b0;
                CS_TST_CMN = 1'b0;
		        ALUControl = 2'b10;
		     end
  	    4'b1100: // ORR
		     begin
			    CS_RSB = 1'b0;
                CS_TST_CMN = 1'b0;
		        ALUControl = 2'b11;
		     end
        4'b1000: //TST <===== NOVO
             begin
                CS_RSB = 1'b0;
                CS_TST_CMN = 1'b1;
                ALUControl = 2'b10;
             end
        4'b1011: //CMN <===== NOVO
             begin
                CS_RSB = 1'b0;
                CS_TST_CMN = 1'b1;
                ALUControl = 2'b00;
             end
        
  	    default: // unimplemented
		     begin
			    CS_RSB = 1'bx;
                CS_TST_CMN = 1'bx;
		        ALUControl = 2'bx;
		     end
      endcase
      // update flags if S bit is set 
	// (C & V only updated for arith instructions)
      FlagW[1]      = Funct[0]; // FlagW[1] = S-bit
	// FlagW[0] = S-bit & (ADD | SUB)
      FlagW[0]      = Funct[0] & 
        (ALUControl == 2'b00 | ALUControl == 2'b01); 
    end else begin
      ALUControl = 2'b00; // add for non-DP instructions
      FlagW      = 2'b00; // don't update Flags
      CS_RSB    = 1'b0;
      CS_TST_CMN    = 1'b0;
    end
              
  // PC Logic
  assign PCS  = ((Rd == 4'b1111) & RegW) | Branch; 
endmodule

module condlogic(input  logic       clk, reset,
                 input  logic [3:0] Cond,
                 input  logic [3:0] ALUFlags,
                 input  logic [1:0] FlagW,
                 input  logic       PCS, RegW, MemW,
                 output logic       PCSrc, RegWrite, MemWrite,
                 input  logic       CS_TST_CMN);
                 
  logic [1:0] FlagWrite;
  logic [3:0] Flags;
  logic       CondEx;

  flopenr #(2)flagreg1(clk, reset, FlagWrite[1], 
                       ALUFlags[3:2], Flags[3:2]);
  flopenr #(2)flagreg0(clk, reset, FlagWrite[0], 
                       ALUFlags[1:0], Flags[1:0]);

  // write controls are conditional
  condcheck cc(Cond, Flags, CondEx);
  assign FlagWrite = FlagW & {2{CondEx}};
  assign RegWrite  = RegW  & CondEx  & ~CS_TST_CMN;
  assign MemWrite  = MemW  & CondEx;
  assign PCSrc     = PCS   & CondEx;
endmodule    

module condcheck(input  logic [3:0] Cond,
                 input  logic [3:0] Flags,
                 output logic       CondEx);
  
  logic neg, zero, carry, overflow, ge;
  
  assign {neg, zero, carry, overflow} = Flags;
  assign ge = (neg == overflow);
                  
  always_comb
    case(Cond)
      4'b0000: CondEx = zero;             // EQ
      4'b0001: CondEx = ~zero;            // NE
      4'b0010: CondEx = carry;            // CS
      4'b0011: CondEx = ~carry;           // CC
      4'b0100: CondEx = neg;              // MI
      4'b0101: CondEx = ~neg;             // PL
      4'b0110: CondEx = overflow;         // VS
      4'b0111: CondEx = ~overflow;        // VC
      4'b1000: CondEx = carry & ~zero;    // HI
      4'b1001: CondEx = ~(carry & ~zero); // LS
      4'b1010: CondEx = ge;               // GE
      4'b1011: CondEx = ~ge;              // LT
      4'b1100: CondEx = ~zero & ge;       // GT
      4'b1101: CondEx = ~(~zero & ge);    // LE
      4'b1110: CondEx = 1'b1;             // Always
      default: CondEx = 1'bx;             // undefined
    endcase
endmodule

module datapath(input  logic        clk, reset,
                input  logic [1:0]  RegSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic        ALUSrc, CS_RSB,
                input  logic [1:0]  ALUControl,
                input  logic        MemtoReg,
                input  logic        PCSrc,
                output logic [3:0]  ALUFlags,
                output logic [31:0] PC,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResult, WriteData,
                input  logic [31:0] ReadData);

  logic [31:0] PCNext, PCPlus4, PCPlus8;
  logic [31:0] ExtImm, SrcA, SrcB, Result;
  logic [3:0]  RA1, RA2;

  // next PC logic
  mux2 #(32)  pcmux(PCPlus4, Result, PCSrc, PCNext);
  flopr #(32) pcreg(clk, reset, PCNext, PC);
  adder #(32) pcadd1(PC, 32'b100, PCPlus4);
  adder #(32) pcadd2(PCPlus4, 32'b100, PCPlus8);

  // register file logic
  mux2 #(4)   ra1mux(Instr[19:16], 4'b1111, RegSrc[0], RA1);
  mux2 #(4)   ra2mux(Instr[3:0], Instr[15:12], RegSrc[1], RA2);
  regfile     rf(clk, RegWrite, RA1, RA2,
                 Instr[15:12], Result, PCPlus8, 
                 SrcA, WriteData); 
  mux2 #(32)  resmux(ALUResult, ReadData, MemtoReg, Result);
  extend      ext(Instr[23:0], ImmSrc, ExtImm);

  // ALU logic
  mux2 #(32)  srcbmux(WriteData, ExtImm, ALUSrc, SrcB);
  alu         alu(SrcA, SrcB, ALUControl, 
                  ALUResult, ALUFlags);
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [3:0]  ra1, ra2, wa3, 
               input  logic [31:0] wd3, r15,
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[14:0];

  // three ported register file
  // read two ports combinationally
  // write third port on rising edge of clock
  // register 15 reads PC+8 instead

  always_ff @(posedge clk)
    if (we3) rf[wa3] <= wd3;	

  assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
  assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];
endmodule

module extend(input  logic [23:0] Instr,
              input  logic [1:0]  ImmSrc,
              output logic [31:0] ExtImm);
 
  always_comb
    case(ImmSrc) 
               // 8-bit unsigned immediate
      2'b00:   ExtImm = {24'b0, Instr[7:0]};  
               // 12-bit unsigned immediate 
      2'b01:   ExtImm = {20'b0, Instr[11:0]}; 
               // 24-bit two's complement shifted branch 
      2'b10:   ExtImm = {{6{Instr[23]}}, Instr[23:0], 2'b00}; 
      default: ExtImm = 32'bx; // undefined
    endcase             
endmodule

module adder #(parameter WIDTH=8)
              (input  logic [WIDTH-1:0] a, b,
               output logic [WIDTH-1:0] y);
             
  assign y = a + b;
endmodule

module flopenr #(parameter WIDTH = 8)
                (input  logic             clk, reset, en,
                 input  logic [WIDTH-1:0] d, 
                 output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)   q <= 0;
    else if (en) q <= d;
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

// alu.sv
// 25 June 2013 David_Harris@hmc.edu, Sarah.Harris@unlv.edu
//
// A 32-bit ALU
module alu(input  logic [31:0] a, b,
           input  logic [1:0]  ALUControl,
           output logic [31:0] Result,
           output logic [3:0]  ALUFlags);

  logic        neg, zero, carry, overflow;
  logic [31:0] condinvb;
  logic [32:0] sum;

  assign condinvb = ALUControl[0] ? ~b : b;
  assign sum = a + condinvb + ALUControl[0];

  always_comb
    casex (ALUControl[1:0])
      2'b0?: Result = sum;
      2'b10: Result = a & b;
      2'b11: Result = a | b;
    endcase

  assign neg      = Result[31];
  assign zero     = (Result == 32'b0);
  assign carry    = (ALUControl[1] == 1'b0) & sum[32];
  assign overflow = (ALUControl[1] == 1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);
  assign ALUFlags = {neg, zero, carry, overflow};
endmodule
