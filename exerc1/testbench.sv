module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
//  wire [3:0] VGA_R;
//  wire [3:0] VGA_G;
//  wire [3:0] VGA_B;
//  wire VGA_HS_O;
//  wire VGA_VS_O;
  

  // instantiate device to be tested
  top dut(clk, MemWrite, VGA_R, VGA_G, VGA_B, VGA_HS_O, VGA_VS_O);
  
  // initialize test
  initial
    begin
      $dumpfile("dump.vcd"); $dumpvars(0);
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        if(DataAdr === 100 & WriteData === 7) begin
          $display("Simulation succeeded");
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end
endmodule
