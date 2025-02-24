/*
 * -----------------------------------------------------------------------
 * File: core.v
 * Created Date: 2025-02-24 11:00:48
 * Author: ChongHao Xu
 * Email: chonghaoxu0109@gmail.com
 * -----
 * Last Modified: 2025-02-24 11:01:10
 * Modified By: ChongHao Xu
 * -----
 * Copyright (c) 2025 Digital IC Design Lab(802) EC,NKUST
 * All rights reserved.
 * -----
 * Description: 
 * 
 * 
 * -----
 * HISTORY:
 * Date      	By  	Comments
 * -----------------------------------------------------------------------
 */
module core(
    input wire Processer_clk,run,rst,
    //instruction Memory
    input wire [31:0] instruction_memory_data,
    output wire [31:0] PC,
    //Memory
    output wire [31:0] Memory_Adder,Memory_datain,
    output wire MemWrite,MemRead,
    input wire [31:0]  Memory_data
  );

  wire clk;
  assign clk = run ? Processer_clk : 1'b0;
  //instruction Memory
  assign PC=IF_PC;


  assign Memory_Adder	=MEM_result;
  assign Memory_datain=MEM_B;
  assign {MemWrite,MemRead}={MEM_MemWrite,MEM_MemRead};


  //----------Wire Declared----------------------
  //IF
  wire [31:0] IF_instruction;
  wire [31:0] IF_PC;
  wire [31:0] IF_PCa4;
  //ID
  wire [31:0] ID_instruction,ID_PC;
  wire [31:0] ID_imm32;
  wire ID_RegDst,ID_ALUsrc,ID_MemtoReg,ID_RegWrite,ID_MemWrite,ID_MemRead;
  wire [1:0] ID_PCstc;
  wire [3:0] ID_ALUop;
  wire [31:0] ID_A,ID_B;
  //EXE
  wire EXE_RegDst,EXE_ALUsrc,EXE_MemtoReg,EXE_RegWrite,EXE_MemWrite,EXE_MemRead;
  wire [1:0] EXE_PCstc;
  wire [3:0] EXE_ALUop;
  wire [31:0] EXE_PC,EXE_A,EXE_B,EXE_ALUB,EXE_imm32;
  wire [25:0] EXE_instruction;
  wire EXE_zero;
  wire [31:0] EXE_result;
  wire [31:0] EXE_t1,EXE_t2;
  wire [4:0] EXE_rw;
  wire [31:0] rA,rB;
  //MEM
  wire MEM_MemtoReg,MEM_RegWrite,MEM_MemWrite,MEM_MemRead;
  wire [31:0] MEM_t1,MEM_t2;
  wire MEM_zero;
  wire [1:0] MEM_PCstc;
  wire [31:0] MEM_result,MEM_B;
  wire [4:0] MEM_rw;
  wire [31:0] MEM_dataout;
  //WB
  wire WB_MemtoReg,WB_RegWrite;
  wire [31:0] WB_result,WB_dataout,WB_busW;
  wire [4:0] WB_rw;
  //Forwards
  wire [1:0] As,Bs;
  //branch Hazard
  wire hazardclr,clr;
  assign hazardclr=rst||clr;

  //---------------------------------------------

  //------------------instruction Fetch-----------------------
  assign IF_PCa4=IF_PC+4;
  IF IF(
       .clk(clk),.rst(rst),
       .instruction_memory_data(instruction_memory_data),
       .PCstc(MEM_PCstc),.zero(MEM_zero),
       .PC(IF_PC),.t0(IF_PCa4),.t1(MEM_t1),.t2(MEM_t2),
       .instruction(IF_instruction)
     );
  IFIDReg IFIDReg(.clk(clk),.rst(hazardclr),.PC(IF_PCa4),.instruction(IF_instruction),.nPC(ID_PC),.ninstruction(ID_instruction));
  //----------------------------------------------------------





  //------------------instruction decode----------------------
  controlunit controlunit(
                .ins(ID_instruction[31:26]),.funct(ID_instruction[5:0]),
                .RegDst(ID_RegDst),.ALUsrc(ID_ALUsrc),.MemtoReg(ID_MemtoReg),
                .RegWrite(ID_RegWrite),.MemWrite(ID_MemWrite),.MemRead(ID_MemRead),
                .PCstc(ID_PCstc),.ALUop(ID_ALUop)
              );

  SignExt SignExt(.imm32(ID_imm32),.imm16(ID_instruction[15:0]));

  gpr gpr(
        .clk(clk),.rst(rst),
        .RegWrite(WB_RegWrite),
        .RW(WB_rw),.RA(ID_instruction[25:21]),.RB(ID_instruction[20:16]),
        .busW(WB_busW),.busA(ID_A),.busB(ID_B)
      );

  IDEXEReg IDEXEReg(
             .clk(clk),.rst(hazardclr),.MemtoReg(ID_MemtoReg),
             .PCstc(ID_PCstc),.RegWrite(ID_RegWrite),.MemWrite(ID_MemWrite),
             .MemRead(ID_MemRead),.ALUop(ID_ALUop),.ALUsrc(ID_ALUsrc),
             .RegDst(ID_RegDst),
             .PC(ID_PC),.A(ID_A),.B(ID_B),.imm32(ID_imm32),
             .instruction26(ID_instruction[25:0]),

             .nPC(EXE_PC),
             .nMemtoReg(EXE_MemtoReg),.nPCstc(EXE_PCstc),
             .nRegWrite(EXE_RegWrite),.nMemWrite(EXE_MemWrite),.nMemRead(EXE_MemRead),
             .nALUop(EXE_ALUop),.nALUsrc(EXE_ALUsrc),.nRegDst(EXE_RegDst),
             .nA(rA),.nB(rB),.nimm32(EXE_imm32),.ninstruction26(EXE_instruction)
           );
  //----------------------------------------------------------





  //----------------------Exeuction---------------------------
  mux1 mux1(.rd(EXE_instruction[15:11]),.rt(EXE_instruction[20:16]),.RegDst(EXE_RegDst),.rw(EXE_rw));
  mux2 mux2(.busB(EXE_B),.imm32(EXE_imm32),.ALUB(EXE_ALUB),.ALUsrc(EXE_ALUsrc));
  ALU ALU(.A(EXE_A),.B(EXE_ALUB),.ALUop(EXE_ALUop),.zero(EXE_zero),.result(EXE_result));
  PCdecoder PCdecoder(.PC(EXE_PC),.imm32(EXE_imm32),.instruction(EXE_instruction),.t1(EXE_t1),.t2(EXE_t2));
  EXEMEMReg EXEMEMReg( .clk(clk),.rst(hazardclr),.MemtoReg(EXE_MemtoReg),.PCstc(EXE_PCstc),.RegWrite(EXE_RegWrite),.MemWrite(EXE_MemWrite),.MemRead(EXE_MemRead),.t1(EXE_t1),.t2(EXE_t2),.zero(EXE_zero),.result(EXE_result),.B(EXE_B),.rw(EXE_rw),.nMemtoReg(MEM_MemtoReg),.nPCstc(MEM_PCstc),.nRegWrite(MEM_RegWrite),.nMemWrite(MEM_MemWrite),.nMemRead(MEM_MemRead),.nt1(MEM_t1),.nt2(MEM_t2),.nzero(MEM_zero),.nresult(MEM_result),.nB(MEM_B),.nrw(MEM_rw));
  //----------------------------------------------------------


  //--------------------Memory Access-------------------------
  //Memory Memory(.clk(clk),.MemWrite(MEM_MemWrite),.MemRead(MEM_MemRead),.rst(rst),.Adder(MEM_result),.datain(MEM_B),.dataout(MEM_dataout));
  MEMWBReg MEMWBReg(.clk(clk),.rst(hazardclr),.MemtoReg(MEM_MemtoReg),.RegWrite(MEM_RegWrite),.result(MEM_result),.dataout(Memory_data),.rw(MEM_rw),
                    .nMemtoReg(WB_MemtoReg),.nRegWrite(WB_RegWrite),.nresult(WB_result),.ndataout(WB_dataout),.nrw(WB_rw));
  //----------------------------------------------------------



  //--------------------Write Back-----------------------------

  mux3 mux3(.result(WB_result),.dataout(WB_dataout),.MemtoReg(WB_MemtoReg),.busW(WB_busW));
  //-----------------------------------------------------------


  //-------------------forwards unit---------------------------

  forwards forwards(	.As(As),.Bs(Bs),
                     .A_addr(EXE_instruction[25:21]),.B_addr(EXE_instruction[20:16]),
                     .MEM_addr(MEM_rw),.WB_addr(WB_rw),
                     .MEM_WB(MEM_RegWrite),.WB_WB(WB_RegWrite)
                   );

  ALU_DataSelect ALU_DataSelectA(.out(EXE_A),.Register(rA),.MEM(MEM_result),.WB(WB_busW),.S(As));
  ALU_DataSelect ALU_DataSelectB(.out(EXE_B),.Register(rB),.MEM(MEM_result),.WB(WB_busW),.S(Bs));
  //-----------------------------------------------------------

  //--------------------branch Hazard--------------------------
  branchHazard branchHazard(.clr(clr),.MEM_PCstc(MEM_PCstc),.MEM_zero(MEM_zero));
  //-----------------------------------------------------------


endmodule














module IF(clk,rst,PCstc,zero,PC,instruction_memory_data,t2,t1,t0,instruction);

  input clk,rst;
  input [1:0] PCstc;
  input zero;
  input [31:0] t0,t1,t2;
  input [31:0] instruction_memory_data;

  output reg [31:0] instruction;

  output reg [31:0] PC;
  reg [31:0] newPC;

  //reg [7:0] instruction_memory [1023:0];
  reg stall;



  always@(*)
  begin
    if(stall)
      instruction=0;
    else
      instruction = instruction_memory_data;
  end

  always@(*)
  begin
    case(PCstc)
      0:
        newPC<=t0;
      1:
      begin
        if(zero==1)
          newPC<=t1;
        else
          newPC<=t0;
      end
      2:
        newPC<=t2;
    endcase
  end

  always@(negedge clk or posedge rst)
  begin
    if(rst)
    begin
      PC = 32'h0000_3000;
      stall=0;
    end
    else if(instruction[31:26]==6'b100011)
      stall=1;
    else
    begin
      PC = newPC;
      stall=0;
    end
  end
endmodule

module mux1(rd,rt,RegDst,rw);
  input [4:0] rd,rt;
  input RegDst;
  output reg[4:0] rw;

  always@(*)
  begin
    if(RegDst)
      rw=rd;
    else
      rw=rt;
  end
endmodule

module SignExt(imm32,imm16);
  input [15:0] imm16;
  output [31:0] imm32;
  assign imm32={{16{imm16[15]}},{imm16}};
endmodule

module gpr(clk,RegWrite,rst,RW,RA,RB,busW,busA,busB);

  input clk,RegWrite,rst;
  input [4:0] RW,RA,RB;
  input [31:0] busW;
  output [31:0] busA,busB;

  reg [31:0] r[31:0];

  //reset
  integer i;
  always@(posedge rst)
  begin
    for(i=0;i<32;i=i+1)
    begin
      r[i]<=0;
    end
  end

  //Set busA & busB
  assign busA=r[RA];
  assign busB=r[RB];

  //Read
  always@(posedge clk)
  begin
    if(RegWrite)
    begin
      r[RW]<=busW;
      r[0]<=0;
    end
  end
endmodule

module mux2(busB,imm32,ALUB,ALUsrc);
  input [31:0] busB,imm32;
  input ALUsrc;
  output reg[31:0] ALUB;

  always@(*)
  begin
    if(ALUsrc)
      ALUB=imm32;
    else
      ALUB=busB;
  end
endmodule

module ALU (A,B,ALUop,zero,result,carryout);

  input [31:0] A,B;
  input [3:0] ALUop;
  output zero;
  output reg [31:0] result;
  output reg carryout;

  wire [31:0] na,nb;
  assign na = ALUop[3] ? ~A : A;
  assign nb = ALUop[2] ? ~B : B;

  always@(*)
  begin
    case(ALUop[1:0])
      2'b00:
        result = na | nb ;
      2'b01:
        result = na & nb ;
      2'b10:
        {carryout,result} = na + nb + ALUop[2]|ALUop[3];
      2'b11:
        result = A < B ? 1 : 0;
    endcase
  end

  assign zero = ~(|result);
endmodule

module Memory(clk,rst,MemWrite,MemRead,Adder,datain,dataout);
  input clk,rst,MemWrite,MemRead;
  input [31:0] Adder;
  input [31:0] datain;
  output [31:0] dataout;

  wire [9:0] pointer;
  assign pointer=Adder[9:0];
  reg [7:0] Mem [1023:0];

  //reset
  integer i;
  always@(posedge rst)
  begin
    for(i=0;i<32;i=i+1)
    begin
      Mem[i]=0;
    end
  end

  //R
  assign dataout={Mem[pointer],Mem[pointer+1],Mem[pointer+2],Mem[pointer+3]};

  //W
  always@(posedge clk)
  begin
    if(MemWrite)
      {Mem[pointer],Mem[pointer+1],Mem[pointer+2],Mem[pointer+3]}<=datain;
    //else if(MemRead) dataout <= {Mem[pointer],Mem[pointer+1],Mem[pointer+2],Mem[pointer+3]};
  end
endmodule

module mux3(result,dataout,MemtoReg,busW);
  input [31:0] result,dataout;
  input MemtoReg;
  output reg[31:0] busW;

  always@(*)
  begin
    if(MemtoReg)
      busW=dataout;
    else
      busW=result;
  end
endmodule


module controlunit(ins,funct,RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,MemRead,PCstc,ALUop);
  input [5:0] ins;
  input [5:0] funct;
  output RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,MemRead;
  output [1:0] PCstc;
  output [3:0] ALUop;

  wire [2:0] preopcode;

  maincontroler c1(ins,RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,MemRead,PCstc,preopcode);
  ALUcontroler c2(preopcode,funct,ALUop);
endmodule


module maincontroler(opcode,RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,Memread,PCstc,preopcode);
  input [5:0] opcode;
  output reg RegDst;
  output reg ALUsrc,MemtoReg,RegWrite,MemWrite,Memread;
  output reg [1:0] PCstc;
  output reg [2:0] preopcode;

  always@(opcode)
  begin
    case(opcode)
      //R-format
      6'b00_0000:
        {RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,Memread,PCstc,preopcode}=11'b100100_00_111;//R-type

      //i-format
      6'b00_1000:
        {RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,Memread,PCstc,preopcode}=11'b010100_00_000;//addi add
      6'b00_1100:
        {RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,Memread,PCstc,preopcode}=11'b010100_00_010;//andi and
      6'b00_1101:
        {RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,Memread,PCstc,preopcode}=11'b010100_00_011;//ori or

      6'b10_0011:
        {RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,Memread,PCstc,preopcode}=11'b011101_00_000;//lw  add
      6'b10_1011:
        {ALUsrc,RegWrite,MemWrite,Memread,PCstc,preopcode}=10'b1010_00_000;//sw  add

      6'b00_0100:
        {ALUsrc,RegWrite,MemWrite,Memread,PCstc,preopcode}=9'b0000_01_001;//beq sub

      //j-format
      6'b00_0010:
        PCstc=2'b10;

      default {RegDst,ALUsrc,MemtoReg,RegWrite,MemWrite,Memread,PCstc,preopcode}=11'b00000_00_000;
    endcase
  end
endmodule



module ALUcontroler(preopcode,funct,ALUop);
  input [2:0] preopcode;
  input [5:0] funct;
  output reg [3:0] ALUop;
  always@(preopcode or funct)
  begin
    casex({preopcode,funct})

      9'b000_xxxxxx:
        ALUop=4'b0010; //add
      9'b001_xxxxxx:
        ALUop=4'b0110; //sub
      9'b010_xxxxxx:
        ALUop=4'b0001; //and
      9'b011_xxxxxx:
        ALUop=4'b0000; //or

      9'b111_100000:
        ALUop=4'b0010; //add
      9'b111_100010:
        ALUop=4'b0110; //sub
      9'b111_100100:
        ALUop=4'b0001; //and
      9'b111_100101:
        ALUop=4'b0000; //or
      9'b111_101010:
        ALUop=4'b0111; //set on less than
    endcase
  end
endmodule

module PCdecoder(PC,imm32,instruction,t1,t2);
  input [31:0] PC,imm32;
  input [25:0] instruction;
  output [31:0] t1,t2;


  assign t1=PC+{{imm32[29:0]},{2'b00}};
  assign t2={PC[31:28],instruction,2'b00};
endmodule



//----------------Pipeline------------------
module IFIDReg(
    input clk,rst,
    input [31:0] PC,instruction,
    output reg [31:0] nPC,ninstruction
  );
  always@(posedge clk or posedge rst)
  begin
    if(clk)
      {nPC,ninstruction} = {PC,instruction};
    else
      {nPC,ninstruction} = 0;
  end

endmodule

module IDEXEReg(
    input clk,rst,
    //--------------controler-------------
    input MemtoReg,
    input [1:0] PCstc,
    input [3:0] ALUop,
    input RegWrite,MemWrite,MemRead,ALUsrc,RegDst,

    output reg nMemtoReg,
    output reg [1:0] nPCstc,
    output reg nRegWrite,nMemWrite,nMemRead,nALUsrc,nRegDst,
    output reg [3:0] nALUop,
    //-------------------------------------

    input [31:0] PC,A,B,imm32,
    input [25:0] instruction26,
    output reg [31:0] nPC,nA,nB,nimm32,
    output reg [25:0] ninstruction26
  );

  always@(posedge clk or posedge rst)
  begin
    if(clk)
    begin
      {nMemtoReg,nPCstc,nRegWrite,nMemWrite,nMemRead,nALUop,nALUsrc,nRegDst} = {MemtoReg,PCstc,RegWrite,MemWrite,MemRead,ALUop,ALUsrc,RegDst};
      {nPC,nA,nB,nimm32,ninstruction26} <= {PC,A,B,imm32,instruction26};
    end
    else
    begin
      {nMemtoReg,nPCstc,nRegWrite,nMemWrite,nMemRead,nALUop,nALUsrc,nRegDst} = 0;
      {nPC,nA,nB,nimm32,ninstruction26} = 0;
    end
  end
endmodule

module EXEMEMReg(
    input clk,rst,
    //--------------controler-------------
    input MemtoReg,
    input [1:0] PCstc,
    input RegWrite,MemWrite,MemRead,

    output reg nMemtoReg,
    output reg [1:0] nPCstc,
    output reg nRegWrite,nMemWrite,nMemRead,
    //-------------------------------------
    input [31:0] t1,t2,
    input zero,
    input [31:0] result,B,
    input [4:0] rw,

    output reg [31:0] nt1,nt2,
    output reg nzero,
    output reg [31:0] nresult,nB,
    output reg [4:0] nrw
  );

  always@(posedge clk or posedge rst)
  begin
    if(clk)
    begin
      {nMemtoReg,nPCstc,nRegWrite,nMemWrite,nMemRead} <= {MemtoReg,PCstc,RegWrite,MemWrite,MemRead};
      {nt1,nt2,nzero,nresult,nB,nrw} <= {t1,t2,zero,result,B,rw};
    end
    else
    begin
      {nMemtoReg,nPCstc,nRegWrite,nMemWrite,nMemRead} <= 0;
      {nt1,nt2,nzero,nresult,nB,nrw} <= 0;
    end
  end
endmodule

module MEMWBReg(
    input clk,rst,
    //--------------controler-------------
    input MemtoReg,RegWrite,

    output reg nMemtoReg,nRegWrite,
    //-------------------------------------
    input [31:0] result,dataout,
    input [4:0] rw ,
    output reg [31:0] nresult,ndataout,
    output reg [4:0] nrw
  );
  always@(posedge clk or posedge rst)
  begin
    if(clk)
    begin
      {nMemtoReg,nRegWrite} = {MemtoReg,RegWrite};
      {nresult,ndataout,nrw} = {result,dataout,rw};
    end
    else
    begin
      {nMemtoReg,nRegWrite} = 0;
      {nresult,ndataout,nrw} = 0;
    end
  end
endmodule
//------------------------------------------






//----------------Forwards------------------

//Register Select s=0;
//Mem      Select s=1;
//WB       Select s=2;
module forwards(As,Bs,A_addr,B_addr,MEM_addr,WB_addr,MEM_WB,WB_WB);
  output reg [1:0] As,Bs;
  input [4:0] A_addr,B_addr,MEM_addr,WB_addr;
  input MEM_WB,WB_WB;

  always@(*)
  begin
    if((A_addr==MEM_addr)&&(MEM_WB))
      As=1;
    else if((A_addr==WB_addr)&&(WB_WB))
      As=2;
    else
      As=0;

    if((B_addr==MEM_addr)&&(MEM_WB))
      Bs=1;
    else if((B_addr==WB_addr)&&(WB_WB))
      Bs=2;
    else
      Bs=0;
  end
endmodule

module ALU_DataSelect(out,Register,MEM,WB,S);
  output reg [31:0] out;
  input [31:0] Register,MEM,WB;
  input [1:0] S;

  always@(*)
  begin
    case(S)
      0:
        out<=Register;
      1:
        out<=MEM;
      2:
        out<=WB;
      default out<=Register;
    endcase
  end
endmodule
//------------------------------------------
module branchHazard(clr,MEM_PCstc,MEM_zero);
  output reg clr;
  input [1:0] MEM_PCstc;
  input MEM_zero;

  always@(*)
  begin
    case({MEM_PCstc,MEM_zero})
      3'b011:
        clr=1;
      3'b101:
        clr=1;
      default clr=0;
    endcase
  end
endmodule
