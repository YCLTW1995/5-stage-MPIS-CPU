`timescale 1ns / 1ps
/*******************************************************************
 * Create Date: 	2016/05/03
 * Design Name: 	Pipeline CPU
 * Module Name:		Pipe_CPU 
 * Project Name: 	Architecture Project_3 Pipeline CPU
 
 * Please DO NOT change the module name, or your'll get ZERO point.
 * You should add your code here to complete the project 3.
 ******************************************************************/
module Pipe_CPU(
        clk_i,
		rst_i
		);
    
/****************************************
*            I/O ports                  *
****************************************/
input clk_i;
input rst_i;

/****************************************
*          Internal signal              *
****************************************/
wire [31:0] beforeifid ,afteraddpc,toidex ,instruction;
wire [31:0] ReadData1,ReadData2 ;
wire [2:0] ALUop ;
wire regdst,branch,memread,memtoreg,memwrite,alusrc,regwrite;
wire [31:0] aftersignextend ;
wire [70:0]memwbout;

/**** IF stage ****/
//control signal...


/**** ID stage ****/
//control signal...


/**** EX stage ****/
//control signal...


/**** MEM stage ****/
//control signal...


/**** WB stage ****/
//control signal...


/**** Data hazard ****/
//control signal...


/****************************************
*       Instantiate modules             *
****************************************/
//Instantiate the components in IF stage
wire [31:0] connectpc ;
wire [31:0] pcout ;


wire memexzero ;
wire [31:0] memexaddr ;
wire [4:0] memexnumregdst;
wire [31:0] memexalu ,memexdata;
wire [152:0] beforeidex ;
wire [152:0] idexout ;
wire [1:0]forwardA,forwardB ;
wire [31:0] alusrc1 ,alusrc2,decidealusrc2_1,afteralu;	 
wire   [3:0] aluctr ;
wire zero ;
wire [4:0]numregdst ,memctr;
wire [31:0] aftershift ,addaddr;
wire [31:0] memdataout;
wire	ans ;
wire [31:0] aftermemwbmux ;

  MUX_2to1 #(.size(32)) BefPCMux(
  .data0_i(afteraddpc),
  .data1_i(memexaddr),
  .select_i(ans),
  .data_o(connectpc)
        );

ProgramCounter PC(
  .clk_i(clk_i),
  .rst_i (rst_i),
  .pc_in_i(connectpc),
  .pc_out_o(pcout)
        );

Instr_Memory IM(
  .pc_addr_i(pcout),
  .instr_o(beforeifid)
	    );

Adder Add_pc(
  .src1_i(pcout),
  .src2_i(4),
  .sum_o(afteraddpc)
		);
Pipe_Reg #(.size(32)) IF_ID(
        .rst_i(rst_i), .clk_i(clk_i), .data_i(beforeifid[31:0]), .data_o(instruction[31:0])
	);

Pipe_Reg #(.size(32)) IF_ID_2(
        .rst_i(rst_i), .clk_i(clk_i), .data_i(afteraddpc), .data_o(toidex)
	);
	
//Instantiate the components in ID stage
Reg_File RF(
  .clk_i(clk_i),
  .rst_i(rst_i),
  .RSaddr_i(instruction[25:21]),
  .RTaddr_i(instruction[20:16]),
  .RDaddr_i(memwbout[70:66]),
  .RDdata_i(aftermemwbmux[31:0]),
  .RegWrite_i(memwbout[1]),
  .RSdata_o(ReadData1),
  .RTdata_o(ReadData2)
		);

Decoder Control(
	.instr_op_i(instruction[31:26]),
	.RegWrite_o(regwrite),
	.ALU_op_o(ALUop),
	.ALUSrc_o(alusrc),
	.RegDst_o(regdst),
	.Branch_o(branch),
	.MemWrite_o(memwrite),
	.MemRead_o(memread),
	.MemtoReg_o(memtoreg)
		);

Sign_Extend Sign_Extend(
	.data_i(instruction[15:0]),
	.data_o(aftersignextend)
		);	

assign beforeidex = {afteraddpc,instruction[15:11],instruction[20:16],instruction[25:21],aftersignextend,ReadData2,ReadData1,ALUop,alusrc,regdst,branch,memwrite,memread,regwrite,memtoreg} ;

Pipe_Reg #(.size(153)) ID_EX(
	.clk_i(clk_i),
	.rst_i (rst_i),
	.data_i(beforeidex),
	.data_o(idexout)
		);
		
//Instantiate the components in EX stage



ALU ALU(
  .src1_i(alusrc1),
  .src2_i(alusrc2),
  .ctrl_i(aluctr),
  .result_o(afteralu),
  .zero_o(zero)
		);

ALU_Ctrl ALU_Control(
  .funct_i(idexout[79:74]),
  .ALUOp_i(idexout[9:7]),
  .ALUCtrl_o(aluctr)
		);
MUX_3to1 #(.size(32)) ALUMux1(
  .data0_i(idexout[41:10]),
  .data1_i(aftermemwbmux),
  .data2_i(memexalu),
  .select_i(forwardA),
  .data_o(alusrc1)
  );

MUX_3to1 #(.size(32)) ALUMux2(
  .data0_i(idexout[73:42]),
  .data1_i(aftermemwbmux),
  .data2_i(memexalu),
  .select_i(forwardB),
  .data_o(decidealusrc2_1)
  );

MUX_2to1 #(.size(32)) ALUinput2_Mux(
  .data0_i(decidealusrc2_1),
  .data1_i(idexout[105:74]),
  .select_i(idexout[6]),
  .data_o(alusrc2)
        );

MUX_2to1 #(.size(5)) RegDst_Mux(
  .data0_i(idexout[115:111]),
  .data1_i(idexout[120:116]),
  .select_i(idexout[5]),
  .data_o(numregdst)
        );

Shift_Left_Two_32 Shifter(
  .data_i(idexout[105:74]),
  .data_o(aftershift)
    );

Adder Adder_Addr(
  .src1_i(idexout[152:121]),
  .src2_i(aftershift),
  .sum_o(addaddr)
    );
ForwardinUnit forward(
  .EX_MEMRegWrite(memctr[1]),
  .MEM_WBRegWrite(memwbout[1]),
  .EX_MEMRegisterRd(memexnumregdst),
  .MEM_WBRegisterRd(memwbout[70:66]),
  .ID_EXRegisterRs(idexout[110:106]),
  .ID_EXRegisterRt(idexout[115:111]),
  .ForwardA(forwardA),
  .ForwardB(forwardB)
  );
  
  
Pipe_Reg #(.size( 1)) EXMEM_Zero(
	.rst_i(rst_i), .clk_i(clk_i), .data_i(zero), .data_o(memexzero)
	);			
		
Pipe_Reg #(.size(32)) EXMEM_Addr(
	.rst_i(rst_i), .clk_i(clk_i), .data_i(addaddr), .data_o(memexaddr)
	);

Pipe_Reg #(.size( 5)) EXMEM_Regdst(
	.rst_i(rst_i), .clk_i(clk_i), .data_i(numregdst), .data_o(memexnumregdst)
	);
		
Pipe_Reg #(.size(32)) EXMEM_Alu(
	.rst_i(rst_i), .clk_i(clk_i), .data_i(afteralu), .data_o(memexalu)
	);

Pipe_Reg #(.size(32)) EXMEM_Data(
	.rst_i(rst_i), .clk_i(clk_i), .data_i(decidealusrc2_1), .data_o(memexdata)
	);		
		
Pipe_Reg #(.size( 5)) EXMEM_Ctr(
	.rst_i(rst_i), .clk_i(clk_i), .data_i(idexout[4:0]), .data_o(memctr)
	);		   
//Instantiate the components in MEM stage

Data_Memory DM(
  .clk_i(clk_i),
  .rst_i(rst_i),
  .addr_i(memexalu),
  .data_i(memexdata),
  .MemRead_i(memctr[2]),
  .MemWrite_i(memctr[3]),
  .data_o(memdataout)
	    );

assign ans = memctr[4] & memexzero;
Pipe_Reg #(.size(71)) MEMWB(
  .clk_i(clk_i),
  .rst_i (rst_i),
  .data_i({memexnumregdst,memexalu,memdataout,memctr[1],memctr[0]}),
  .data_o(memwbout)
		);


//Instantiate the components in WB stage

MUX_2to1 #(.size(32)) Mux_AfterMemWb(
  .data0_i(memwbout[65:34]),
  .data1_i(memwbout[33:2]),
  .select_i(memwbout[0]),
  .data_o(aftermemwbmux[31:0])
        );

/****************************************
*         Signal assignment             *
****************************************/
	
endmodule

