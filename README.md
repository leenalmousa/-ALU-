
module subtractor(o,b,a,c);
//this will act as a subtractor  (a-c)
    input [15:0]a;//  the first number which we will subtract from (a)
    input [15:0]c;// the number which we  will subtract (c)
    wire  [15:0]x;// the number  -c( we transformed the number c to it negative  by taking the second complement) 
    wire [15:0]carry;// will contain the carry of each previous addition 
    output  [2:0]o;// o[0]: will contain  one when overflowing o[1]:will return 1 when the output is  zero ando[2: will produce one when the addition produces a carry
    output [15:0]b;// the output
    wire h;
        xor(x[0],c[0],1'b1);
        xor(x[1],c[1],1'b1);
        xor(x[2],c[2],1'b1);
        xor(x[3],c[3],1'b1);
        xor(x[4],c[4],1'b1);
        xor(x[5],c[5],1'b1);
        xor(x[6],c[6],1'b1);
        xor(x[7],c[7],1'b1);
        xor(x[8],c[8],1'b1);
        xor(x[9],c[9],1'b1);
        xor(x[10],c[10],1'b1);
        xor(x[11],c[11],1'b1);
        xor(x[12],c[12],1'b1);
        xor(x[13],c[13],1'b1);
        xor(x[14],c[14],1'b1);
        xor(x[15],c[15],1'b1);
        fulladder f0 (b[0],carry[0],a[0],1'b1,x[0]);
        fulladder f1 (b[1],carry[1],a[1],carry[0],x[1]);
        fulladder f2 (b[2],carry[2],a[2],carry[1],x[2]);
        fulladder f3 (b[3],carry[3],a[3],carry[2],x[3]);
        fulladder f4 (b[4],carry[4],a[4],carry[3],x[4]);
        fulladder f5 (b[5],carry[5],a[5],carry[4],x[5]);
        fulladder f6 (b[6],carry[6],a[6],carry[5],x[6]);
        fulladder f7 (b[7],carry[7],a[7],carry[6],x[7]);
        fulladder f8 (b[8],carry[8],a[8],carry[7],x[8]);
        fulladder f9 (b[9],carry[9],a[9],carry[8],x[9]);
        fulladder f10 (b[10],carry[10],a[10],carry[9],x[10]);
        fulladder f11 (b[11],carry[11],a[11],carry[10],x[11]);
        fulladder f12(b[12],carry[12],a[12],carry[11],x[12]);
        fulladder f13(b[13],carry[13],a[13],carry[12],x[13]);
        fulladder f14(b[14],carry[14],a[14],carry[13],x[14]);
        fulladder f15(b[15],o[2],a[15],carry[14],x[15]);
        
        or16bititself o1(h,b);// to figure out if it is  zero or not 
        not(o[1],h);
        xor( o[0],carry[14],o[2]);
endmodule



module or16bititself(o,c);
  input [15:0]c;
  wire [15:0]x;
  output o;
     or(x[0],c[0],c[1]);
     or(x[1],c[2],x[0]);
     or(x[2],c[3],x[1]);
     or(x[3],c[4],x[2]);
     or(x[4],c[5],x[3]);
     or(x[5],c[6],x[4]);
     or(x[6],c[7],x[5]);
     or(x[7],c[8],x[6]);
     or(x[8],c[9],x[7]);
     or(x[9],c[10],x[8]);
     or(x[10],c[11],x[9]);
     or(x[11],c[12],x[10]);
     or(x[12],c[13],x[11]);
     or(x[13],c[14],x[12]);
     or(o,c[15],x[13]);
  endmodule


  
module ram(o,data_out,clk,data_in,address,RDWR);
input clk;
input [15:0] data_in;           // the data which will be written in the memory
input [9:0]address;             // the address of which we will read or write in the memory
output reg [15:0]data_out;      // the data read of the memory
reg[15:0]MEMO[0:1024];          //creation of the memory
input RDWR;                     // the selection line to read  if 1 and write if 0
output [2:0]o;                  // o[1]:will return 1 when the output is  zero
wire h;
integer i;
  initial                       //fillingg the ROM initially  with zero
    begin
     data_out<=0;
     for(  i=0; i<1024;i=i+1 )
         MEMO[i]=0;
    end 
 always@(negedge clk)
 begin 
    if(RDWR==1'b0)
        MEMO[address]<=data_in;
    else if(RDWR==1'b1) begin
        data_out<=MEMO[address];
        end 
 
 end
      or16bititself o1(h,data_out);
      not(o[1],h);
endmodule
   
   
module inncrement(o,b,a);
    input [15:0]a;
    output [15:0]b;
    wire [15:0]carry;
    output  [2:0] o;
        bitadder b1 (o,b,1'b1,a);
endmodule

module bitadder(o,b,c,a);
 //adds 2 numbers c+a
    input [15:0]a;
    input [15:0]c;
    output [15:0]b;
    wire [15:0]carry;// will contain the carry of each previous addition 
     wire [15:0]x;
    output [2:0]o; //o[0]: will contain  one when overflowing o[1]:will return 1 when the output is  zero ando[2: will produce one when the addition produces a carry
   wire h;
        fulladder f0 (b[0],carry[0],a[0],1'b0,c[0]);
        fulladder f1 (b[1],carry[1],a[1],carry[0],c[1]);
        fulladder f2 (b[2],carry[2],a[2],carry[1],c[2]);
        fulladder f3 (b[3],carry[3],a[3],carry[2],c[3]);
        fulladder f4 (b[4],carry[4],a[4],carry[3],c[4]);
        fulladder f5 (b[5],carry[5],a[5],carry[4],c[5]);
        fulladder f6 (b[6],carry[6],a[6],carry[5],c[6]);
        fulladder f7 (b[7],carry[7],a[7],carry[6],c[7]);
        fulladder f8 (b[8],carry[8],a[8],carry[7],c[8]);
        fulladder f9 (b[9],carry[9],a[9],carry[8],c[9]);
        fulladder f10 (b[10],carry[10],a[10],carry[9],c[10]);
        fulladder f11 (b[11],carry[11],a[11],carry[10],c[11]);
        fulladder f12(b[12],carry[12],a[12],carry[11],c[12]);
        fulladder f13(b[13],carry[13],a[13],carry[12],c[13]);
        fulladder f14(b[14],carry[14],a[14],carry[13],c[14]);
        fulladder f15(b[15],o[2],a[15],carry[14],c[15]);
                 
       or16bititself o1(h,b);
      not(o[1],h);
        xor( o[0],carry[14],o[2]);
    
endmodule

   module fulladder(sum,carry,a,b,c);
input a,b,c;
    output sum, carry;
        
        wire x,y,z;//0,1,1
        xor(x,a,b);//0
        and(z,a,b);//1
        and(y,x,c);//0
        xor(sum,x,c);//1
        or(carry,z,y);//1
endmodule
 
module decrement(o,b,a);
    input [15:0]a;
    output [15:0]b;
    output  [2:0] o;
        subtractor b1 (o,b,a,1'b1);
endmodule


   

   
    module  regfile (out1,out2,in1,in2,data_in,RDWR);
    input [2:0]in1;
    input [2:0]in2;
    input RDWR;
    input [15:0]data_in;
    output reg [15:0]out1;
    output reg [15:0]out2;
    reg [15:0]MEM[7:0];
    initial
    begin
    MEM[0]=0;
    MEM[1]=1;
    MEM[2]=2;
    MEM[3]=3;
    MEM[4]=4;
    MEM[5]=5;
    MEM[6]=6;
    MEM[7]=7;
    end
    
    always@(in1 or in2 or RDWR or data_in)
    begin
    
        if(RDWR==1'b0)
            begin
                case(in1)
                3'b000:MEM[0]<=data_in;
                3'b001:MEM[1]<=data_in;
                3'b010:MEM[2]<=data_in;
                3'b011:MEM[3]<=data_in;
                3'b100:MEM[4]<=data_in;
                3'b101:MEM[5]<=data_in;
                3'b110:MEM[6]<=data_in;
                3'b111:MEM[7]<=data_in;
               endcase;
            end
        else 
            begin
                case(in1)
                3'b000:out1<=MEM[0];
                3'b001:out1<=MEM[1];
                3'b010:out1<=MEM[2];
                3'b011:out1<=MEM[3];
                3'b100:out1<=MEM[4];
                3'b101:out1<=MEM[5];
                3'b110:out1<=MEM[6];
                3'b111:out1<=MEM[7];
               endcase;
                 case(in2)
                3'b000:out2<=MEM[0];
                3'b001:out2<=MEM[1];
                3'b010:out2<=MEM[2];
                3'b011:out2<=MEM[3];
                3'b100:out2<=MEM[4];
                3'b101:out2<=MEM[5];
                3'b110:out2<=MEM[6];
                3'b111:out2<=MEM[7];
               endcase;
            end
        
    end
    endmodule
    module left_shift(input1,out,o);
 
output [15:0] out;
input  [15:0] input1;
 output [2:0] o;
 wire h;
 or(out[0],input1[15],input1[15]);
 or(out[1],input1[0],input1[0]);
 or(out[2],input1[1],input1[1]);
 or(out[3],input1[2],input1[2]);
 or(out[4],input1[3],input1[3]);
 or(out[5],input1[4],input1[4]);
 or(out[6],input1[5],input1[5]);
 or(out[7],input1[6],input1[6]);
 or(out[8],input1[7],input1[7]);
 or(out[9],input1[8],input1[8]);
 or(out[10],input1[9],input1[9]);
 or(out[11],input1[10],input1[10]);
 or(out[12],input1[11],input1[11]);
 or(out[13],input1[12],input1[12]);
 or(out[14],input1[13],input1[13]);
 or(out[15],input1[14],input1[14]);
   or16bititself o1(h,out);
      not(o[1],h);
endmodule
 module multiplexer_14_1(X, A0, A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13, S);
   parameter WIDTH=16;  
   output reg[WIDTH-1:0] X;//OUTPUT
   input [WIDTH-1:0]  A0;  
   input [WIDTH-1:0]  A1;  
   input [WIDTH-1:0]  A2;  
   input [WIDTH-1:0]  A3;  
   input [WIDTH-1:0]  A4;  
   input [WIDTH-1:0]  A5;  
   input [WIDTH-1:0]  A6;  
   input [WIDTH-1:0]  A7;  
   input [WIDTH-1:0]  A8;  
   input [WIDTH-1:0]  A9;  
   input [WIDTH-1:0]  A10;  
   input [WIDTH-1:0]  A11;
   input [WIDTH-1:0]  A12;  
   input [WIDTH-1:0]  A13;  
   input 	    [3:0]  S;
    always@(*)begin
   case(S)
      4'b0000:X<=A0;
      4'b0001: X<=A1;
      4'b0010: X<=A2;
      4'b0011: X<=A3;
        //  werent able to complete the set on
      4'b0110: X<=A6;
      4'b0111: X<=A7;
      4'b1000: X<=A8;
      4'b1001: X<=A9;
      4'b1010: X<=A10;
      4'b1011: X<=A11;
      4'b1100: X<=A12;
      4'b1101: X<=A13;
  endcase
end
endmodule

module myClock (clk);
output reg clk;
initial clk=0;
    always 
    #1 clk =~clk;
endmodule
module store_flg_mod(store_flg,operation);
output store_flg;
input [3:0]operation;
wire nt_3,nt_1,nt_flg;
    not(nt_3, operation[3]);
    not(nt_1, operation[1]);
    and(nt_flg,nt_3,nt_1,operation[2],operation[0]);
    not(store_flg,nt_flg);
endmodule


module load_flg_mod(load_flg,operation);
output load_flg;
input [3:0]operation;
wire nt_3,nt_0,nt_flg;
    not(nt_3, operation[3]);
    not(nt_0, operation[0]);
    and(nt_flg,nt_3,nt_0,operation[2],operation[1]);
    not(load_flg,nt_flg);
endmodule

module ALU (reg1,reg2,operation,result,status);
    input [2:0]reg1,reg2;
    input [3:0]operation;
    output[15:0] result;
    output[2:0]status;
    wire clk;
    wire store_flag,load_flag;
    wire [15:0]register1;
    wire [15:0]register2;
    wire [15:0]a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13;
    wire [2:0]s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13;
    
    myClock clk1(clk);
    regfile  regfile1(register1,register2,reg1,reg2,a6,load_flag);
    inncrement i1(s0, a0, register1);
    decrement d1(s1, a1, register1);
    subtractor ss(s2, a2, register1, register2);
    bitadder b1(s3,a3, register1, register2);
    store_flg_mod sfx1(store_flag,operation);
    load_flg_mod lfx1(load_flag,operation);
    ram ram1(s6,a6, clk, register1, register2, store_flag);
 
    //#1 $display ("%b %b ",result_t,status_t);
    multiplexer_14_1 result_mux(result, a0, a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13, operation);
    multiplexer_14_1 status_mux(status, s0, s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13, operation);

endmodule
module test;
reg [2:0]reg1_t; 
reg [2:0]reg2_t;
reg clk;
//always #1 clk_t =~clk_t;
reg  [3:0]operation_t;
wire [15:0] result_t;
wire [2:0]status_t;
 ALU  a1  (reg1_t,reg2_t,operation_t,result_t,status_t);
initial 
begin 
// case 0
reg1_t= 2   ; 
//reg2_t= 0101010101010101;
 
operation_t=0 ;
#1 $display ("Increment Reg 2: %b %b ",result_t,status_t);

// case 1
reg1_t= 1  ; 
//reg2_t= 2;

operation_t=1 ;
#2 $display ("Decrement Reg 1: %b %b  ",result_t,status_t);

// case 2
reg1_t=3   ; 
reg2_t=1;

operation_t=2 ;
#3 $display ("Subtraction Reg 3-1: %b %b ",result_t,status_t);


// case 3
reg1_t=5   ; 
reg2_t=6 ;
// we werent able to complete the comparoter
operation_t=3 ;
#4 $display ("Addition  Reg 5+6: %b %b  ",result_t,status_t);


// case 4
reg1_t=3  ; 
reg2_t=5 ;

operation_t=5 ;
#5 $display ("store value in reg (3)  to mem [5] %b %b ",a1.ram1.MEMO[5],status_t);

// case 5
reg1_t= 4  ; 
reg2_t= 5;

operation_t=6 ;
#6 $display ("load from mem[5] save the value in reg[4]  %b %b ",a1.regfile1.MEM[4],status_t);
// case 6
reg1_t= 2  ; 
reg2_t=3 ;


#5 $finish;
 end  
  endmodule
