
//-------------------------------------------------------------------------
// www.verificationguide.com testbench.sv
//-------------------------------------------------------------------------
//tbench_top or testbench top, this is the top most file, in which DUT(Design Under Test) and
Verification environment are connected.
//-------------------------------------------------------------------------
//including interfcae and testcase files
`include "interface.sv"
//-------------------------[NOTE]---------------------------------
//Particular testcase can be run by uncommenting, and commenting the rest
//`include "random_test.sv"
//`include "wr_rd_test.sv"
`include "default_rd_test.sv"
//----------------------------------------------------------------
module tbench_top;

 //clock and reset signal declaration
 bit clk;
 bit reset;

 //clock generation
 always #5 clk = ~clk;

 //reset Generation
 initial begin
 reset = 1;
 #5 reset =0;
 end


 //creatinng instance of interface, inorder to connect DUT and testcase
 mem_intf intf(clk,reset);

 //Testcase instance, interface handle is passed to test as an argument
 test t1(intf);

 //DUT instance, interface signals are connected to the DUT ports
 memory DUT (
 .clk(intf.clk),
 .reset(intf.reset),
 .addr(intf.addr),
 .wr_en(intf.wr_en),
 .rd_en(intf.rd_en),
 .wdata(intf.wdata),
 .rdata(intf.rdata)
 );

 //enabling the wave dump
 initial begin
 $dumpfile("dump.vcd"); $dumpvars;
 end
endmodule
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
class transaction;
 //declaring the transaction items
 rand bit [1:0] addr;
 rand bit wr_en;
 rand bit rd_en;
 rand bit [7:0] wdata;
 bit [7:0] rdata;
 bit [1:0] cnt;

 //constaint, to generate any one among write and read
 constraint wr_rd_c { wr_en != rd_en; };

 //postrandomize function, displaying randomized values of items
 function void post_randomize();
 $display("--------- [Trans] post_randomize ------");
 $display("\t addr = %0h",addr);
 if(wr_en) $display("\t wr_en = %0h\t wdata = %0h",wr_en,wdata);
 if(rd_en) $display("\t rd_en = %0h",rd_en);
 $display("-----------------------------------------");
 endfunction

 //deep copy method
 function transaction do_copy();
 transaction trans;
 trans = new();
 trans.addr = this.addr;
 trans.wr_en = this.wr_en;
 trans.rd_en = this.rd_en;
 trans.wdata = this.wdata;
 return trans;
 endfunction
endclass
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
class generator;

 //declaring transaction class
 rand transaction trans,tr;

 //repeat count, to specify number of items to generate
 int repeat_count;

 //mailbox, to generate and send the packet to driver
 mailbox gen2driv;

 //event
 event ended;

 //constructor
 function new(mailbox gen2driv,event ended);
 //getting the mailbox handle from env, in order to share the transaction packet between the
generator and driver, the same mailbox is shared between both.
 this.gen2driv = gen2driv;
 this.ended = ended;
 trans = new();
 endfunction

 //main task, generates(create and randomizes) the repeat_count number of transaction packets and
puts into mailbox
 task main();
 repeat(repeat_count) begin
 if( !trans.randomize() ) $fatal("Gen:: trans randomization failed");
 tr = trans.do_copy();
 gen2driv.put(tr);
 end
 -> ended;
 endtask

endclass
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
//gets the packet from generator and drive the transaction paket items into interface (interface is
connected to DUT, so the items driven into interface signal will get driven in to DUT)
`define DRIV_IF mem_vif.DRIVER.driver_cb
class driver;

 //used to count the number of transactions
 int no_transactions;

 //creating virtual interface handle
 virtual mem_intf mem_vif;

 //creating mailbox handle
 mailbox gen2driv;

 //constructor
 function new(virtual mem_intf mem_vif,mailbox gen2driv);
 //getting the interface
 this.mem_vif = mem_vif;
 //getting the mailbox handles from environment
 this.gen2driv = gen2driv;
 endfunction

 //Reset task, Reset the Interface signals to default/initial values
 task reset;
 wait(mem_vif.reset);
 $display("--------- [DRIVER] Reset Started ---------");
 `DRIV_IF.wr_en <= 0;
 `DRIV_IF.rd_en <= 0;
 `DRIV_IF.addr <= 0;
 `DRIV_IF.wdata <= 0;
 wait(!mem_vif.reset);
 $display("--------- [DRIVER] Reset Ended ---------");
 endtask

 //drivers the transaction items to interface signals
 task drive;
 transaction trans;
 `DRIV_IF.wr_en <= 0;
 `DRIV_IF.rd_en <= 0;
 gen2driv.get(trans);
 $display("--------- [DRIVER-TRANSFER: %0d] ---------",no_transactions);
 @(posedge mem_vif.DRIVER.clk);
 `DRIV_IF.addr <= trans.addr;
 if(trans.wr_en) begin
 `DRIV_IF.wr_en <= trans.wr_en;
 `DRIV_IF.wdata <= trans.wdata;
 $display("\tADDR = %0h \tWDATA = %0h",trans.addr,trans.wdata);
 @(posedge mem_vif.DRIVER.clk);
 end
 if(trans.rd_en) begin
 `DRIV_IF.rd_en <= trans.rd_en;
 @(posedge mem_vif.DRIVER.clk);
 `DRIV_IF.rd_en <= 0;
 @(posedge mem_vif.DRIVER.clk);
 trans.rdata = `DRIV_IF.rdata;
 $display("\tADDR = %0h \tRDATA = %0h",trans.addr,`DRIV_IF.rdata);
 end
 $display("-----------------------------------------");
 no_transactions++;
 endtask


 //
 task main;
 forever begin
 fork
 //Thread-1: Waiting for reset
 begin
 wait(mem_vif.reset);
 end
 //Thread-2: Calling drive task
 begin
 forever
 drive();
 end
 join_any
 disable fork;
 end
 endtask

endclass
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
`include "transaction.sv"
`include "generator.sv"
`include "driver.sv"
class environment;

 //generator and driver instance
 generator gen;
 driver driv;

 //mailbox handle's
 mailbox gen2driv;

 //event for synchronization between generator and test
 event gen_ended;

 //virtual interface
 virtual mem_intf mem_vif;

 //constructor
 function new(virtual mem_intf mem_vif);
 //get the interface from test
 this.mem_vif = mem_vif;

 //creating the mailbox (Same handle will be shared across generator and driver)
 gen2driv = new();

 //creating generator and driver
 gen = new(gen2driv,gen_ended);
 driv = new(mem_vif,gen2driv);
 endfunction

 //
 task pre_test();
 driv.reset();
 endtask

 task test();
 fork
 gen.main();
 driv.main();
 join_any
 endtask

 task post_test();
 wait(gen_ended.triggered);
 wait(gen.repeat_count == driv.no_transactions);
 endtask

 //run task
 task run;
 pre_test();
 test();
 post_test();
 $finish;
 endtask

endclass
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
`include "environment.sv"
program test(mem_intf intf);

 //declaring environment instance
 environment env;

 initial begin
 //creating environment
 env = new(intf);

 //setting the repeat count of generator as 4, means to generate 4 packets
 env.gen.repeat_count = 4;

 //calling run of env, it interns calls generator and driver main tasks.
 env.run();
 end
endprogram
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
interface mem_intf(input logic clk,reset);

 //declaring the signals
 logic [1:0] addr;
 logic wr_en;
 logic rd_en;
 logic [7:0] wdata;
 logic [7:0] rdata;

 //driver clocking block
 clocking driver_cb @(posedge clk);
 default input #1 output #1;
 output addr;
 output wr_en;
 output rd_en;
 output wdata;
 input rdata;
 endclocking

 //monitor clocking block
 clocking monitor_cb @(posedge clk);
 default input #1 output #1;
 input addr;
 input wr_en;
 input rd_en;
 input wdata;
 input rdata;
 endclocking

 //driver modport
 modport DRIVER (clocking driver_cb,input clk,reset);

 //monitor modport
 modport MONITOR (clocking monitor_cb,input clk,reset);

endinterface
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
`include "environment.sv"
program test(mem_intf intf);

 class my_trans extends transaction;

 bit [1:0] count;

 function void pre_randomize();
 wr_en.rand_mode(0);
 rd_en.rand_mode(0);
 addr.rand_mode(0);

 if(cnt %2 == 0) begin
 wr_en = 1;
 rd_en = 0;
 addr = count;
 end
 else begin
 wr_en = 0;
 rd_en = 1;
 addr = count;
 count++;
 end
 cnt++;
 endfunction

 endclass

 //declaring environment instance
 environment env;
 my_trans my_tr;

 initial begin
 //creating environment
 env = new(intf);

 my_tr = new();

 //setting the repeat count of generator as 4, means to generate 4 packets
 env.gen.repeat_count = 10;

 env.gen.trans = my_tr;

 //calling run of env, it interns calls generator and driver main tasks.
 env.run();
 end
endprogram
//-------------------------------------------------------------------------
// www.verificationguide.com
//-------------------------------------------------------------------------
`include "environment.sv"
program test(mem_intf intf);

 class my_trans extends transaction;

 bit [1:0] count;

 function void pre_randomize();
 wr_en.rand_mode(0);
 rd_en.rand_mode(0);
 addr.rand_mode(0);
 wr_en = 0;
 rd_en = 1;
 addr = cnt;
 cnt++;
 endfunction

 endclass

 //declaring environment instance
 environment env;
 my_trans my_tr;

 initial begin
 //creating environment
 env = new(intf);

 my_tr = new();

 //setting the repeat count of generator as 4, means to generate 4 packets
 env.gen.repeat_count = 4;

 env.gen.trans = my_tr;

 //calling run of env, it interns calls generator and driver main tasks.
 env.run();
 end
endprogram
####################################################################
module ram
#(
parameter MEM_DW = 32,
parameter MEM_AW = 16,
)
(
input clk;
input mem_sel;
input mem_rw; //1'b0: write , 1'b1 : read
input [MEM_DW-1:0]mem_wd;
input [MEM_AW-1:0]mem_addr;
output [MEM_DW-1:0]mem_rd;
)
logic [MEM_DW-1:0] mem [1<<MEM_AW-1:0];
always_ff @(posedge clk) begin
if(mem_sel & ~mem_rw) begin
mem[mem_addr]= mem_wd;
end
end
assign mem_rd = (mem_sel & mem_rw) ? mem[mem_addr] : {MEM_DW{1'b0}};
endmodule
// Code your testbench here
// or browse Examples
module top;
localparam MEM_DW = 8;
localparam MEM_AW = 16;
logic clk;
logic mem_sel;
logic mem_rw; //1'b0: write , 1'b1 : read
 logic [MEM_DW-1:0]mem_wd;
 logic [MEM_AW-1:0]mem_addr;
logic [MEM_DW-1:0]mem_rd;
initial begin
 clk=1'b0;
 forever begin
 #10 clk = ~clk;
 end
end
initial begin
 repeat(5) @(posedge clk);
 mem_sel =1'b0;mem_rw = 1'b0;
 repeat(1) @(posedge clk);
 mem_sel =1'b1;mem_rw = 1'b0;mem_addr = 8'h23;mem_wd = 16'h2136;
 repeat(1) @(posedge clk);
 mem_sel =1'b0;mem_rw = 1'b0;mem_addr = 8'hxx;mem_wd = 16'hxx;
 repeat(1) @(posedge clk);
 mem_sel =1'b1;mem_rw = 1'b1;mem_addr = 8'h23;
 repeat(1) @(posedge clk);
 mem_sel =1'b0;mem_rw = 1'b1;mem_addr = 8'hxx;
 repeat(5) @(posedge clk);
 $display("completing the simulation");
 $finish;
end
initial begin
 $dumpfile("top.vcd");
 $dumpvars(0,top);
end
//ram dut(.*);
 ram #(MEM_DW,MEM_AW) dut(.*);
endmodule
//////////driver
// Code your testbench here
// or browse Examples
module top;
localparam MEM_DW = 8;
localparam MEM_AW = 5;
logic clk;
logic mem_sel;
logic mem_rw; //1'b0: write , 1'b1 : read
 logic [MEM_DW-1:0]mem_wd;
 logic [MEM_AW-1:0]mem_addr;
logic [MEM_DW-1:0]mem_rd;
initial begin
 clk=1'b0;
 forever begin
 #10 clk = ~clk;
 end
end
task mem_control_init;
begin
 mem_sel =1'b0;mem_rw = 1'b0;
end
endtask
task mem_write;
 input [MEM_AW-1:0]addr;
 input [MEM_DW-1:0]data;
 begin
 mem_sel =1'b1;mem_rw = 1'b0;mem_addr = addr;mem_wd = data;
 repeat(1) @(posedge clk);
 mem_sel =1'b0;mem_rw = 1'b0;mem_addr = 5'hxx;mem_wd = 8'hxx;
 end
endtask
task mem_read;
 input [MEM_AW-1:0]addr;
 begin
 mem_sel =1'b1;mem_rw = 1'b1;mem_addr = addr;
 repeat(1) @(posedge clk);
 mem_sel =1'b0;mem_rw = 1'b0;mem_addr = 5'hxx;
 end
endtask
initial begin
 mem_control_init();
 repeat(5) @(posedge clk);
 mem_write(5'h07,8'hAB);
 repeat(5) @(posedge clk);
 mem_read(5'h07);
 repeat(5) @(posedge clk);
 $finish;
end
initial begin
 $dumpfile("top.vcd");
 $dumpvars(0,top);
end
//ram dut(.*);
 ram #(MEM_DW,MEM_AW) dut(.*);
endmodule
module top;
reg [1:0] state;
parameter zero=0, one=1, two=2, three=3;
always @(state)
begin
 case (state)
zero:
out = 4'b0000;
one:
out = 4'b0001;
two:
out = 4'b0010;
three:
out = 4'b0100;
default:
out = 4'b0000;
endcase
end
endmodule
//verilog code for 11 sequence detector
module one_one_Detector(clk, rst, din,dout)
input clk, rst;
input din;
output reg dout;
parameter a=00 , b =01, c=10;
reg [1:0]ns, [1:0]ps;
always@(posedge clk)
if(rst)
dout <= 0;
ns <= a;
else
ps <= ns;
always@(din,ps)
case(ps): begin
'a':
if ns <= (din == 0)? a:b;
dout <= 0;
'b':
if ns <= (din == 0)? a:c;
dout <= 0;
'c':
if ns <= (din == 0)? a:c;
dout <= 1;
'default':
 ns <= a;
dout <= 0;
end
endcase
endmodule
module dff(q,qbar,clk, rst,din);
input clk, rst;
input din;
output reg q;
oupput qbar;
assign qbar = ~q;
always@(posedge clk)
begin
if(~rst)
q <= 0;
else
q <= din;
end
endmodule
module async_dff(q,qbar,clk,rst,din)
input clk, rst, din;
output reg q, qbar;
always@(posedge clk or negedge rst)
begin
if(~rst)
q <= 1'b0;
else
q <= din;
end
endmodule
module dff_tb(q,qbar,clk, rst,din);
reg clk,rst,din;
wire q,qbar;
dff dut(.q(q),.qbar(qbar),.clk(clk), .rst(rst),.din(din));
initial
begin
//initialisation
clk = 0;
rst = 0;
din = 0;
//wait for 200ns
#200
rst = 1;
d = 1;
#20;
forever #50 din = ~din;
end
always #10 clk = ~clk;
end
