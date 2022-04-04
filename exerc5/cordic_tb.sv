`timescale 1 ns/100 ps

module cordic_test;

localparam  SZ = 16; // precisão de bits

reg  [SZ-1:0] Xin, Yin;
reg  [31:0] angle;
wire [SZ:0] Xout, Yout;
reg         CLK_100MHZ;

//Gerador de ondas

localparam FALSE = 1'b0;
localparam TRUE = 1'b1;

localparam VALUE = 32000/1.647; // Como o ganho do sistema é de 1,647, realiza-se a divisão do valor por ele

reg signed [63:0] i;
reg      start;

initial
begin
   start = FALSE;
   $write("Starting sim");
   CLK_100MHZ = 1'b0;
   angle = 0;
   Xin = VALUE;
   Yin = 1'd0;
	i=60;

   #1000;
   @(posedge CLK_100MHZ);
   start = TRUE;

    for (i = 0; i < 360; i = i + 1)     // pega ângulos de entrada de 0 a 359° com incremento de 1°
   //for (i = 30; i < 60; i = i + 30)     // increment by 30 degrees only
   begin
      @(posedge CLK_100MHZ);
      start = FALSE;
      angle = ((1 << 32)*i)/360;    // example: 45 deg = 45/360 * 2^32 = 32'b00100000000000000000000000000000 = 45.000 degrees -> atan(2^0)
     $display ("angulo = %d, %h",i, angle);
   end

   #500
  $write("Fim da execução");
   $stop;
end

 sine_cosine cordic(CLK_100MHZ, angle, Xin, Yin, Xout, Yout);

parameter CLK100_SPEED = 10;  // 100Mhz = 10nS

initial
begin
  $dumpfile("dump.vcd");
  $dumpvars;
  
  CLK_100MHZ = 1'b0;
  $display ("CLK_100MHZ started");
  #5;
  forever
  begin
    #(CLK100_SPEED/2) CLK_100MHZ = 1'b1;
    #(CLK100_SPEED/2) CLK_100MHZ = 1'b0;
  end
end

endmodule
