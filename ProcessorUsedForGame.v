module skeleton(inclock, ps2_clock, ps2_data,
				   seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8, XMBOut,
					j,jal,jr,flush,DXinstructionOut,branchControl,MWinstructionOut, ALUinBsig, ALUinAsig, ALUop, 
					nSX,destReg,writeData,
					PCin,stallOut, PCout, dataRegA, dataRegB, ledArray_red, ledArray_yel, zeroLED, de2red, de2yellow);

					
					
	input 			inclock;
	inout 			ps2_data, ps2_clock;
	
	output [8:0] zeroLED;
	output 	[6:0] 	seg1, seg2, seg3, seg4, seg5, seg6, seg7, seg8;
	output stallOut, branchControl, flush, j,jal,jr;
	output [31:0] writeData, MWinstructionOut, DXinstructionOut;
	output [4:0] destReg, ALUop;
	output[31:0] nSX,XMBOut;
	output[31:0] dataRegA, dataRegB;
	output[2:0] ALUinBsig, ALUinAsig;
	output [8:0] ledArray_red, ledArray_yel;
	output [11:0] PCin, PCout;
	wire [31:0] ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15;

	wire	[7:0]	ps2_key_data;
	wire			ps2_key_pressed;
	wire	[7:0]	ps2_out;
	wire resetn;
	output [8:0] de2red, de2yellow;
	assign de2red = ledArray_red;
	assign de2yellow = ledArray_yel;
	assign resetn = 1'b0;
	
	pll div(inclock,clock);
	// UNCOMMENT FOLLOWING LINE AND COMMENT ABOVE LINE TO RUN AT 50 MHz
	//assign clock = inclock/2;

	assign zeroLED = 9'b000000000;
	assign seg3 = 7'b1111111;
	assign seg4 = 7'b1111111;
	assign seg5 = 7'b1111111;
	assign seg6 = 7'b1111111;
	assign seg7 = 7'b1111111;
	assign seg2 = 7'b1111111;
	
	
	// your processor
	processor myprocessor(XMBOut,j,jal,jr,flush,DXinstructionOut,branchControl,
								 MWinstructionOut, ALUinBsig, ALUinAsig, ALUop, nSX,destReg,writeData, 
								 clock, ~resetn,PCin,stallOut, PCout, ps2_key_pressed, ps2_out, dataRegA, dataRegB, 
								 seg8, seg1, ledArray_red, ledArray_yel);

	// keyboard controller
	PS2_Interface myps2(clock, ~resetn, ps2_clock, ps2_data, ps2_key_data, ps2_key_pressed, ps2_out);

	// example for sending ps2 data to the first two seven segment displays
	//Hexadecimal_To_Seven_Segment hex1(ps2_out[3:0], seg6);
	//Hexadecimal_To_Seven_Segment hex2(ps2_out[7:4], seg7);
endmodule

module processor(XMBOut,j,jal,jr,flush,DXinstructionOut,branchControl,
									MWinstructionOut, ALUinBsig, ALUinAsig, ALUop, nSX, destReg,writeData,
									clock, reset,PCin,stallOut, PCout, ps2_key_pressed, ps2_out, dataRegA,dataRegB, p1_score, p2_score, ledArray_red, ledArray_yel);// ro16, ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15);

	input 			clock, reset, ps2_key_pressed;
	input 	[7:0]	ps2_out;

	//output 	[31:0] 	lcd_data;

	// GRADER OUTPUTS - YOU MUST CONNECT TO YOUR DMEM
	//output 	[31:0] 	debug_data;
	output [11:0] PCin, PCout;
	output stallOut, branchControl, flush, j,jal,jr;// enablewrite;
	output [31:0] writeData, MWinstructionOut, DXinstructionOut;
	output [4:0] destReg, ALUop;
	output[31:0] nSX,XMBOut;
	output[31:0] dataRegA, dataRegB;
	output [6:0] p1_score, p2_score;
	output [8:0] ledArray_red, ledArray_yel;
	output[2:0] ALUinBsig, ALUinAsig;
	//output [31:0] ALUinA, ALUinB, ALUresult;
	wire [4:0] ctrlregA, ctrlregB;
	wire enablewrite;
	wire [31:0] ro16;
	wire [31:0] ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15;
	
	wire [31:0] inro5, inro6, inro7, inro8, inro9, inro10, inro11, inro12, inro13, inro14, inro15, inro16;
	assign ro5 = inro5;
	assign ro6 = inro6;
	assign ro7 = inro7;
	assign ro8 = inro8;
	assign ro9 = inro9;
	assign ro10 = inro10;
	assign ro11 = inro11;
	assign ro12 = inro12;
	assign ro13 = inro13;
	assign ro14 = inro14;
	assign ro15 = inro15;
	assign ro16 = inro16;
	
	// your processor here
	//
	//wire[11:0] PCin,PCout;
	//wire PCintrue;
	//assign PCintrue = (stallOut) ? PCout : PCin;
	assign enablewrite = (RegWrite | jal);
	
	game_output tictactoe(inro7, inro8, inro9, inro10, inro11, inro12, inro13, inro14, inro15, inro5, inro6, ledArray_red, ledArray_yel, p1_score, p2_score);
	wire[11:0] PCoutfinal;
	assign PCoutfinal = (flush) ? 12'b000000000000 : PCout;
	my_register12 PCregister(~stallOut,PCin,PCout,clock, ~reset);
	//my_register12 PCregister(1'b1,PCin,PCout,clock, ~reset, stallOut);

	wire[31:0] instruction;
	wire[11:0] PCplus1;
	wire[31:0] nop;

	FDstage myFD(PCoutfinal,instruction,PCplus1, (clock));
	assign nop = 32'b00000000000000000000000000000000;

	wire[31:0] FDAOut,FDBOut, FDinstructionOut, FDinstructionIn, FDinstructionIntermed;
	wire[11:0] FDPCOut;

	assign FDinstructionIntermed = (flush) ? nop : instruction;
	assign FDinstructionIn = (stallOut) ? FDinstructionOut : FDinstructionIntermed;
	Iregister FDreg(PCplus1,32'b00000000000000000000000000000000,32'b00000000000000000000000000000000,FDinstructionIn,FDPCOut,FDAOut,FDBOut,FDinstructionOut,(clock&(~stallOut)),~reset);

	//wire[31:0] dataRegA,dataRegB;
	DXstage myDX(FDinstructionOut,(RegWrite | jal),destReg,writeData,dataRegA,dataRegB,clock,~reset, inro5, inro6, inro7, inro8, inro9, inro10, inro11, inro12, inro13, inro14, inro15, inro16, ps2_key_pressed, ps2_out, ctrlregA, ctrlregB);

	wire[31:0] DXAOut,DXBOut,DXinstrIn;//, DXinstructionOut;
	wire[11:0] DXPCOut;
	assign DXinstrIn = (flush|stallOut) ? nop : FDinstructionOut;
	Iregister DXreg(FDPCOut,dataRegA,dataRegB,DXinstrIn,DXPCOut,DXAOut,DXBOut,DXinstructionOut,clock,~reset);

	//wire [31:0] ALUinB, ALUinA, nSX;
	wire[31:0]  ALUinA, ALUinB, ALUresult;
	wire isNotEqual,isLessThan, bne, beq;//, j, jal, jr;
	mux3 ALUinAmux(ALUinAsig,XMAOut,writeData,DXAOut,ALUinA);
	mux3 ALUinBmux(ALUinBsig,XMAOut,writeData,DXBOut,ALUinB);
	//wire[4:0] ALUop;
	wire[31:0] operandB;
	XMstage myXM(DXinstructionOut,ALUinA,ALUinB,ALUresult,isNotEqual,isLessThan, bne, beq, j, jal, jr, nSX,ALUop, operandB );

	wire[15:0] PC1plusN;
	wire[15:0] DXPCforAdding;
	assign DXPCforAdding[11:0] = DXPCOut;
	assign DXPCforAdding[15:12] = 4'b0000;
	add12 incrementPC(DXPCforAdding,nSX[15:0],1'b0,PC1plusN);

	//wire branchControl;
	assign branchControl = (bne & isNotEqual) | (beq & ~isNotEqual);

	wire[11:0] PCintermediate, PCint2;
	assign PCintermediate = (j | jal) ? DXinstructionOut[11:0] :(PCplus1);

	assign PCint2 = (jr) ? ALUinB[11:0] : PCintermediate;
	assign PCin = (branchControl) ? PC1plusN[11:0] : PCint2;

	wire[31:0] XMAOut,XMinstructionOut,XMBOut, XMinstructionIn;
	wire[11:0] XMPCOut;

	Iregister XMreg(DXPCOut,ALUresult,ALUinB,XMinstructionIn,XMPCOut,XMAOut,XMBOut,XMinstructionOut,clock,~reset);

	wire[31:0] dmemOut, d4dMem;
	//wire flush;
	assign flush = (branchControl | j | jal | jr);

	assign d4dMem = (dindMemsig) ? writeData : XMBOut;
	MWstage myMW(XMinstructionOut,XMAOut,d4dMem,dmemOut,clock);

	wire[31:0] MWAOut,MWBOut;//,MWinstructionOut;
	wire[11:0] MWPCOut;

	Iregister MWreg(XMPCOut,XMAOut,dmemOut,XMinstructionOut,MWPCOut,MWAOut,MWBOut,MWinstructionOut,clock,~reset);

	wire MemtoReg, RegWrite;
	writeBackControl writeBack(MWinstructionOut[31:27],MemtoReg,RegWrite);

	wire[31:0] writeDataint;// writeData;
	//wire[4:0] destReg;
	assign writeData = (MemtoReg) ? MWBOut : MWAOut;
	//assign writeData = (jal) ? DXPCOut : writeDataint;
	assign destReg = MWinstructionOut[26:22];
	assign XMinstructionIn[31:27] = DXinstructionOut[31:27];
	assign XMinstructionIn[21:0] = DXinstructionOut[21:0];
	assign XMinstructionIn[26:22] = (jal) ? 5'b11111 : DXinstructionOut[26:22];

	wire dindMemsig;
	//wire[2:0] ALUinAsig, ALUinBsig;
	bypassLogic bypass(DXinstructionOut,XMinstructionOut,MWinstructionOut,ALUinAsig,ALUinBsig,dindMemsig);

//	wire stallOut;
	stallLogic stall(DXinstructionOut,FDinstructionOut,stallOut);

	////// THIS IS REQUIRED FOR GRADING
	// CHANGE THIS TO ASSIGN YOUR DMEM WRITE ADDRESS ALSO TO debug_addr
	//assign debug_addr = (XMAOut);
	// CHANGE THIS TO ASSIGN YOUR DMEM DATA INPUT (TO BE WRITTEN) ALSO TO debug_data
	//assign debug_data = (d4dMem);
	////////////////////////////////////////////////////////////

endmodule

module stallLogic(DX,FD,stallOut);
	input[31:0] DX,FD;
	output stallOut;

	wire[4:0] DXIROP, FDIRRS, FDIRR2, FDIROP, DXIRRD;

	wire R;
	assign R = (~FD[31]) & (~FD[30]) & (~FD[29]) & (~FD[28]) & (~FD[27]);
	assign FDIRR2 = (R) ? FD[16:12] : FD[26:22];

	assign DXIROP = DX[31:27];
	assign FDIRRS = FD[21:17];
	assign FDIROP = FD[31:27];
	assign DXIRRD = DX[26:22];

	wire DXIROPisLOAD, FDIRRSisDXIRRD, FDIRR2isDXIRRD, FRIROPisSTORE;

	assign DXIROPisLOAD = (DXIROP[4] ^~ 1'b0) & (DXIROP[3] ^~ 1'b1) & (DXIROP[2] ^~ 1'b0) & (DXIROP[1] ^~ 1'b0) & (DXIROP[0] ^~ 1'b0);
	assign FDIRRSisDXIRRD = (DXIRRD[4] ^~ FDIRRS[4]) & (DXIRRD[3] ^~ FDIRRS[3]) & (DXIRRD[2] ^~ FDIRRS[2]) &(DXIRRD[1] ^~ FDIRRS[1]) & (DXIRRD[0] ^~ FDIRRS[0]);
	assign FDIRR2isDXIRRD = (DXIRRD[4] ^~ FDIRR2[4]) &(DXIRRD[3] ^~ FDIRR2[3]) & (DXIRRD[2] ^~ FDIRR2[2]) & (DXIRRD[1] ^~ FDIRR2[1]) & (DXIRRD[0] ^~ FDIRR2[0]);
	assign FRIROPisSTORE = (FDIROP[4] ^~ 1'b0) & (FDIROP[3] ^~ 1'b0) & (FDIROP[2] ^~ 1'b1) & (FDIROP[1] ^~ 1'b1) & (FDIROP[0] ^~ 1'b1);

	assign stallOut = (DXIROPisLOAD) & (((FDIRRSisDXIRRD) | (FDIRR2isDXIRRD)) & (~FRIROPisSTORE));

endmodule

module bypassLogic(DX,XM,MW,ALUinAsig,ALUinBsig,dindMemsig);
	input[31:0] DX,XM,MW;
	output[2:0] ALUinAsig, ALUinBsig;
	output dindMemsig;

	wire lwXM,addiXM,RXM,lwMW,addiMW,RMW, R;

	wire[4:0] DXIRRS, XMIRRD, MWIRRD, DXIRR2;
	wire XMwrites, MWwrites;
	assign DXIRRS = DX[21:17];
	assign XMIRRD = XM[26:22];
	assign MWIRRD = MW[26:22];

	assign lwXM = ~XM[31] & XM[30] & ~XM[29] & ~XM[28] & ~XM[27];
	assign addiXM = ~XM[31] & ~XM[30] & XM[29] & ~XM[28] & XM[27];
	assign RXM = ~XM[31] & ~XM[30] & ~XM[29] & ~XM[28] & ~XM[27];

	assign XMwrites = lwXM | addiXM | RXM;

	assign lwMW = ~MW[31] & MW[30] & ~MW[29] & ~MW[28] & ~MW[27];
	assign addiMW = ~MW[31] & ~MW[30] & MW[29] & ~MW[28] & MW[27];
	assign RMW = ~MW[31] & ~MW[30] & ~MW[29] & ~MW[28] & ~MW[27];

	assign MWwrites = lwMW | addiMW | RMW;

	assign R = ~DX[31] & ~DX[30] & ~DX[29] & ~DX[28] & ~DX[27];
	assign DXIRR2 = (R) ? DX[16:12] : DX[26:22];

	wire DXIRRSisXMIRRD, DXIRRSisMWIRRD;

	assign DXIRRSisXMIRRD = XMwrites & (DXIRRS[4] ^~ XMIRRD[4]) & (DXIRRS[3] ^~ XMIRRD[3]) &  (DXIRRS[2] ^~ XMIRRD[2]) & (DXIRRS[1] ^~ XMIRRD[1]) &  (DXIRRS[0] ^~ XMIRRD[0]);
	assign DXIRRSisMWIRRD = MWwrites & (DXIRRS[4] ^~ MWIRRD[4]) & (DXIRRS[3] ^~ MWIRRD[3]) &  (DXIRRS[2] ^~ MWIRRD[2]) & (DXIRRS[1] ^~ MWIRRD[1]) &  (DXIRRS[0] ^~ MWIRRD[0]);

	assign ALUinAsig[0] = DXIRRSisXMIRRD;
	assign ALUinAsig[1] = DXIRRSisMWIRRD;
	assign ALUinAsig[2] = ~(DXIRRSisXMIRRD | DXIRRSisMWIRRD);

	wire DXIRR2isXMIRRD, DXIRR2isMWIRRD;

	assign DXIRR2isXMIRRD = XMwrites & (DXIRR2[4] ^~ XMIRRD[4]) & (DXIRR2[3] ^~ XMIRRD[3]) &  (DXIRR2[2] ^~ XMIRRD[2]) & (DXIRR2[1] ^~ XMIRRD[1]) &  (DXIRR2[0] ^~ XMIRRD[0]);
	assign DXIRR2isMWIRRD = MWwrites & (DXIRR2[4] ^~ MWIRRD[4]) & (DXIRR2[3] ^~ MWIRRD[3]) &  (DXIRR2[2] ^~ MWIRRD[2]) & (DXIRR2[1] ^~ MWIRRD[1]) &  (DXIRR2[0] ^~ MWIRRD[0]);

	assign ALUinBsig[0] = DXIRR2isXMIRRD;
	assign ALUinBsig[1] = DXIRR2isMWIRRD;
	assign ALUinBsig[2] = ~(DXIRR2isXMIRRD | DXIRR2isMWIRRD);

	wire XMIRRDisMWIRRD;
	assign XMIRRDisMWIRRD = MWwrites & (MWIRRD[4] ^~ XMIRRD[4]) & (MWIRRD[3] ^~ XMIRRD[3]) &  (MWIRRD[2] ^~ XMIRRD[2]) & (MWIRRD[1] ^~ XMIRRD[1]) &  (MWIRRD[0] ^~ XMIRRD[0]);

	assign dindMemsig = (XMIRRDisMWIRRD);

endmodule

module mux3(signal,in1,in2,in3,out);
	input [2:0] signal;
	input[31:0] in1,in2,in3;
	output[31:0] out;

	wire[31:0] intermediate;
	assign intermediate = (signal[0]) ?  in1 : in2;
	assign out = (signal[2]) ? in3 : intermediate;

endmodule

module writeBackControl(op,MemtoReg,RegWrite);
	input[4:0] op;
	output MemtoReg, RegWrite;

	wire lw,addi,R;

	assign lw = ~op[4] & op[3] & ~op[2] & ~op[1] & ~op[0];
	assign R = ~op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0];
	assign addi = ~op[4] & ~op[3] & op[2] & ~op[1] & op[0];

	assign MemtoReg = lw;
	assign RegWrite = R | lw | addi;

endmodule

module FDstage(PC,instruction,PCplus1,clock);
	input clock;
	input[11:0] PC;
	output[11:0] PCplus1;
	output[31:0] instruction;

	imem myimem(	.address 	(PC),
					.clken		(1'b1),
					.clock		(clock),
					.q			(instruction) // change where output q goes...
	);

	wire[15:0] PCforAdding,PCplus;
	assign PCforAdding[11:0] = PC;
	assign PCforAdding[15:12] = 4'b0000;

	add12 incrementPC(PCforAdding,15'b000000000000001,1'b0,PCplus);
	assign PCplus1 = PCplus[11:0];

endmodule

module DXstage(instruction,writeEnable,destReg,dataToWriteToReg,data_readRegA,data_readRegB,clock,reset, ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15, ro16, ps2_key_pressed, ps2_data, ctrlregA, ctrlregB);
	input clock, reset, writeEnable, ps2_key_pressed;
	input[4:0] destReg;
	input [7:0] ps2_data;
	input[31:0] instruction,dataToWriteToReg;
	output[31:0] data_readRegA, data_readRegB;
	output [31:0] ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15, ro16;
	output [4:0] ctrlregA, ctrlregB;

	wire[4:0] op;
	assign op = instruction[31:27];

	wire R;
	assign R = ~op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0];

	wire[4:0] readRegA,readRegB;
	assign readRegA = instruction[21:17];
	assign readRegB = (R) ? instruction[16:12] : instruction[26:22];
	assign ctrlregA = readRegA;
	assign ctrlregB = readRegB;

	regfile registerFile(clock, writeEnable, reset, destReg, readRegA, readRegB, dataToWriteToReg, data_readRegA, data_readRegB, ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15, ro16, ps2_key_pressed, ps2_data);

endmodule

module XMstage(instruction,A,B,ALUresult,isNotEqual, isLessThan, bne, beq, j, jal, jr, intermediateSX, ALUopOut, operandB);
	input[31:0] instruction, A, B;
	output[31:0] ALUresult;
	output[31:0] intermediateSX;
	output isNotEqual, isLessThan, bne, beq, j, jal, jr;
	output [31:0] operandB;
	output[4:0] ALUopOut;


	wire[4:0] op;
	assign op = instruction[31:27];

	wire branch, ALUSrc;
	wire lw,sw,addi;
	assign lw = ~op[4] & op[3] & ~op[2] & ~op[1] & ~op[0];
	assign sw = ~op[4] & ~op[3] & op[2] & op[1] & op[0];
	assign beq = ~op[4] & ~op[3] & op[2] & op[1] & ~op[0];
	assign bne = ~op[4] & ~op[3] & ~op[2] & op[1] & ~op[0];
	assign addi = ~op[4] & ~op[3] & op[2] & ~op[1] & op[0];
	assign j = ~op[4] & ~op[3] & ~op[2] & ~op[1] & op[0];
	assign jal = ~op[4] & ~op[3] & ~op[2] & op[1] & op[0];
	assign jr = ~op[4] & ~op[3] & op[2] & ~op[1] & ~op[0];

	assign ALUSrc = lw | sw | addi;
	assign branch = beq | bne;

	//wire[31:0] ALUresult;

	wire[4:0] ALUop;
	assign ALUop = instruction[6:2];


	wire[4:0] branchALUop, addALUop;
	assign branchALUop = 5'b00001;
	assign addALUop = 5'b00000;

	wire[4:0] ALUOutintermediate;
	assign ALUOutintermediate = (branch) ? branchALUop : ALUop;
	assign ALUopOut = (addi|sw|lw) ? addALUop : ALUOutintermediate;

	signExtend16to32 mySX(instruction[16:0],intermediateSX);

	assign operandB = (ALUSrc) ?  intermediateSX : B;

	ALU aluProcessor(A, operandB, ALUopOut, instruction[11:7], ALUresult, isNotEqual, isLessThan);

endmodule

module MWstage(instruction,A,B,dmemOut,clock);
	input[31:0] A,B,instruction;
	input clock;
	output[31:0] dmemOut;

	wire[4:0] op;
	wire sw;
	assign op = instruction[31:27];
	assign sw = ~op[4] & ~op[3] & op[2] & op[1] & op[0];

	wire MemWrite;
	assign MemWrite = sw;

	dmem mydmem(	.address	(A[11:0]),
					.clock		(clock),
					.data		(B),
					.wren		(MemWrite),
					.q			(dmemOut) // change where output q goes...
	);

endmodule

module signExtend16to32(in,out);
	input[16:0] in;
	output[31:0] out;

	assign out[31:17] = (in[16]) ? 15'b111111111111111 : 15'b000000000000000;
	assign out[16:0] = in;
endmodule

module add12(a,b,c0,s);
	input [15:0] a,b;
	input c0;
	output [15:0] s;

	wire c8,c16;
	wire P0,P1;
	wire G0,G1;

	assign c8 = G0 | P0 & c0;
	assign c16 = G1 | P1 & c8;

	eightBit_Block block1(a[7:0],b[7:0],c0,P0,G0,s[7:0]);
	eightBit_Block block2(a[15:8],b[15:8],c8,P1,G1,s[15:8]);

endmodule

module Iregister(PC,A,B,instruction,PCOut,AOut,BOut,instructionOut,clock,reset);
	input clock, reset;
	input [11:0] PC;
	input[31:0] A,B,instruction;
	output [11:0] PCOut;
	output [31:0] AOut,BOut,instructionOut;
	wire newReset;

		assign newReset = ~reset;

		genvar g;
		generate
			for (g=0;g<32;g=g+1) begin: loop1
				DFFE a_dffe(.d(instruction[g]),.ena(1'b1),.clk(clock),.prn(1'b1),.clrn(newReset),.q(instructionOut[g]));
			end
		endgenerate

		genvar h;
		generate
			for (h=0;h<32;h=h+1) begin: loop2
				DFFE a_dffe(.d(A[h]),.ena(1'b1),.clk(clock),.clrn(newReset),.prn(1'b1),.q(AOut[h]));
			end
		endgenerate

		genvar i;
		generate
			for (i=0;i<32;i=i+1) begin: loop3
				DFFE a_dffe(.d(B[i]),.ena(1'b1),.clk(clock),.clrn(newReset),.prn(1'b1),.q(BOut[i]));
			end
		endgenerate

		genvar j;
		generate
			for (j=0;j<12;j=j+1) begin: loop4
				DFFE a_dffe(.d(PC[j]),.ena(1'b1),.clk(clock),.clrn(newReset),.prn(1'b1),.q(PCOut[j]));
			end
		endgenerate

endmodule

module control(op,ALUop, branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump,ALUopOut,addi,bne,blt,j,jal,jr,R);
	input[4:0] op, ALUop;
	output branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jump;
	output bne,blt,j,jal,jr,addi,R;
	output[4:0] ALUopOut;

	wire lw,sw,R;
	assign lw = ~op[4] & op[3] & ~op[2] & ~op[1] & ~op[0];
	assign sw = ~op[4] & ~op[3] & op[2] & op[1] & op[0];
	assign blt = ~op[4] & ~op[3] & op[2] & op[1] & ~op[0];
	assign bne = ~op[4] & ~op[3] & ~op[2] & op[1] & ~op[0];
	assign R = ~op[4] & ~op[3] & ~op[2] & ~op[1] & ~op[0];
	assign j = ~op[4] & ~op[3] & ~op[2] & ~op[1] & op[0];
	assign jal = ~op[4] & ~op[3] & ~op[2] & op[1] & op[0];
	assign jr = ~op[4] & ~op[3] & op[2] & ~op[1] & ~op[0];
	assign addi = ~op[4] & ~op[3] & op[2] & ~op[1] & op[0];

	assign ALUSrc = lw | sw | addi;
	assign MemtoReg = lw;
	assign RegWrite = R | lw | addi;
	assign MemRead = lw;
	assign MemWrite = sw;
	assign branch = blt | bne;
	assign Jump = j | jal | jr;

	wire[4:0] branchALUop, addALUop;
	assign branchALUop = 5'b00001;
	assign addALUop = 5'b00000;

	wire[4:0] ALUOutintermediate;
	assign ALUOutintermediate = (branch) ? branchALUop : ALUop;
	assign ALUopOut = (addi|sw|lw) ? addALUop : ALUOutintermediate;

endmodule

module regfile(clock, ctrl_writeEnable, ctrl_Reset, ctrl_writeReg, ctrl_readRegA, ctrl_readRegB, data_writeReg, data_readRegA, data_readRegB, ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15, ro16, ps2_key_pressed, ps2_data);
   input clock, ctrl_writeEnable, ctrl_Reset, ps2_key_pressed;
   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
	input [7:0] ps2_data;
   input [31:0] data_writeReg;
   output [31:0] data_readRegA, data_readRegB, ro5, ro6, ro7, ro8, ro9, ro10, ro11, ro12, ro13, ro14, ro15, ro16;

	//Contains 32-bit decoded signals
	wire [31:0] decoded_writeReg;
	wire [31:0] decoded_readRegA;
	wire [31:0] decoded_readRegB;
	wire [31:0] ps2_data_ext;
	//Decoder blocks with outputs as wires with 32 bits
	decoder32 dc1(ctrl_writeReg, decoded_writeReg);
	decoder32 dc2(ctrl_readRegA, decoded_readRegA);
	decoder32 dc3(ctrl_readRegB, decoded_readRegB);

	wire [31:0] w_out [31:0];
	assign ro5 = w_out[4];
	assign ro6 = w_out[5];
	assign ro7 = w_out[6];
	assign ro8 = w_out[7];
	assign ro9 = w_out[8];
	assign ro10 = w_out[9];
	assign ro11 = w_out[10];
	assign ro12 = w_out[11];
	assign ro13 = w_out[12];
	assign ro14 = w_out[13];
	assign ro15 = w_out[14];
	assign ro16 = w_out[15];

	assign ps2_data_ext = {24'b0, ps2_data};

	my_register reg0(1'b0, 32'b000000000000000000000000000000000, w_out[0], clock, ctrl_Reset);
	my_register reg15(ps2_key_pressed, ps2_data_ext, w_out[15], clock, ctrl_Reset);

	genvar i;
	generate
		for(i=1; i<15; i=i+1) begin:registersLoop1
			my_register regs1(decoded_writeReg[i] & ctrl_writeEnable, data_writeReg, w_out[i], clock, ctrl_Reset);
		end
		for(i=16; i<32; i=i+1) begin:registersLoop2
			my_register regs2(decoded_writeReg[i] & ctrl_writeEnable, data_writeReg, w_out[i], clock, ctrl_Reset);
		end
	endgenerate

	genvar j;
	generate
		for(j=0; j<32; j=j+1) begin:triStates
			assign data_readRegA = decoded_readRegA[j] ? w_out[j] : 32'bZ;
			assign data_readRegB = decoded_readRegB[j] ? w_out[j] : 32'bZ;
		end
	endgenerate
endmodule

//5-bit to 32-bit decoder
module decoder32(binaryInput, decodedSignal);
	input [4:0] binaryInput;
	output [31:0] decodedSignal;
	assign decodedSignal = 1 << binaryInput;
endmodule

module my_RegDecoder(in,out);
	input [4:0] in;
	output [31:0] out;
assign out[0] = ~in[4]&~in[3]&~in[2]&~in[1]&~in[0];
assign out[1] = ~in[4]&~in[3]&~in[2]&~in[1]&in[0];
assign out[2] = ~in[4]&~in[3]&~in[2]&in[1]&~in[0];
assign out[3] = ~in[4]&~in[3]&~in[2]&in[1]&in[0];
assign out[4] = ~in[4]&~in[3]&in[2]&~in[1]&~in[0];
assign out[5] = ~in[4]&~in[3]&in[2]&~in[1]&in[0];
assign out[6] = ~in[4]&~in[3]&in[2]&in[1]&~in[0];
assign out[7] = ~in[4]&~in[3]&in[2]&in[1]&in[0];
assign out[8] = ~in[4]&in[3]&~in[2]&~in[1]&~in[0];
assign out[9] = ~in[4]&in[3]&~in[2]&~in[1]&in[0];
assign out[10] = ~in[4]&in[3]&~in[2]&in[1]&~in[0];
assign out[11] = ~in[4]&in[3]&~in[2]&in[1]&in[0];
assign out[12] = ~in[4]&in[3]&in[2]&~in[1]&~in[0];
assign out[13] = ~in[4]&in[3]&in[2]&~in[1]&in[0];
assign out[14] = ~in[4]&in[3]&in[2]&in[1]&~in[0];
assign out[15] = ~in[4]&in[3]&in[2]&in[1]&in[0];
assign out[16] = in[4]&~in[3]&~in[2]&~in[1]&~in[0];
assign out[17] = in[4]&~in[3]&~in[2]&~in[1]&in[0];
assign out[18] = in[4]&~in[3]&~in[2]&in[1]&~in[0];
assign out[19] = in[4]&~in[3]&~in[2]&in[1]&in[0];
assign out[20] = in[4]&~in[3]&in[2]&~in[1]&~in[0];
assign out[21] = in[4]&~in[3]&in[2]&~in[1]&in[0];
assign out[22] = in[4]&~in[3]&in[2]&in[1]&~in[0];
assign out[23] = in[4]&~in[3]&in[2]&in[1]&in[0];
assign out[24] = in[4]&in[3]&~in[2]&~in[1]&~in[0];
assign out[25] = in[4]&in[3]&~in[2]&~in[1]&in[0];
assign out[26] = in[4]&in[3]&~in[2]&in[1]&~in[0];
assign out[27] = in[4]&in[3]&~in[2]&in[1]&in[0];
assign out[28] = in[4]&in[3]&in[2]&~in[1]&~in[0];
assign out[29] = in[4]&in[3]&in[2]&~in[1]&in[0];
assign out[30] = in[4]&in[3]&in[2]&in[1]&~in[0];
assign out[31] = in[4]&in[3]&in[2]&in[1]&in[0];
endmodule

module my_tri(in,oe,out,clock);
	input clock, oe;
	input [31:0] in;
	output [31:0] out;

	//assign out = (oe & (~clock)) ? in : 32'bz;
	assign out = oe ? in : 32'bz;
endmodule

module my_register(regCtrl,writeData, outData, clock, reset);
	input clock, reset, regCtrl;
	input [31:0] writeData;
	output [31:0] outData;
	wire newReset;

		genvar g;
		generate
			for (g=0;g<32;g=g+1) begin: loop3
				DFFE a_dffe(.d(writeData[g]),.ena(regCtrl),.clk(clock),.clrn(newReset),.q(outData[g]));
			end
		endgenerate

		assign newReset = ~reset;

endmodule

module my_register12(regCtrl,writeData, outData, clock, reset);
//module my_register12(regCtrl,writeData, outData, clock, reset, stall);
	input clock, reset, regCtrl; //stall;
	input [11:0] writeData;
	output [11:0] outData;
	wire newReset;

		genvar g;
		generate
			for (g=0;g<12;g=g+1) begin: loop3
				//DFFE a_dffe(.d(writeData[g]),.ena(regCtrl),.clk(clock),.prn(1'b1 & (~stall)),.clrn(newReset),.q(outData[g]));
				DFFE a_dffe(.d(writeData[g]),.ena(regCtrl),.clk(clock),.prn(1'b1),.clrn(newReset),.q(outData[g]));
			end
		endgenerate

		assign newReset = ~reset;

endmodule
//////////////////////////////////////////////////////////////////////////////
module ALU(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan);
   input [31:0] data_operandA, data_operandB;
   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;
   output [31:0] data_result;
   output isNotEqual, isLessThan;

	wire [7:0] wiresToOps;
	wire [31:0] addOut, subOut, andOut, orOut, sllOut, sraOut, multOut, divOut;
	wire status, data_inputRDY, data_resultRDY;

	my_ALUdecoder decoder(ctrl_ALUopcode,wiresToOps);

	my_adder adder(data_operandA, data_operandB, 1'b0, addOut);
	my_sub subber(data_operandA,data_operandB,1'b0,subOut,isNotEqual,isLessThan);
	my_and ander(data_operandA,data_operandB,andOut);
	my_or orer(data_operandA,data_operandB,orOut);
	my_sll sller(data_operandA,ctrl_shiftamt,sllOut);
	my_sra sraer(data_operandA,ctrl_shiftamt,sraOut);
  	multdiv mult(data_operandA, data_operandB, 1'b1, 1'b0, 1'b1, multOut, status, data_inputRDY, data_resultRDY);
	divider div(data_operandA,data_operandB,divOut, status);

	my_tri triBufferAdd(addOut,wiresToOps[0],data_result);
	my_tri triBufferSub(subOut,wiresToOps[1],data_result);
	my_tri triBufferAnd(andOut,wiresToOps[2],data_result);
	my_tri triBufferOr(orOut,wiresToOps[3],data_result);
	my_tri triBufferSll(sllOut,wiresToOps[4],data_result);
	my_tri triBufferSra(sraOut,wiresToOps[5],data_result);
	my_tri triBufferMult(multOut,wiresToOps[6],data_result);
	my_tri triBufferDiv(divOut,wiresToOps[7],data_result);


endmodule

module my_ALUdecoder(in,out);
	input [4:0] in;
	output [7:0] out;
assign out[0] = ~in[4]&~in[3]&~in[2]&~in[1]&~in[0];
assign out[1] = ~in[4]&~in[3]&~in[2]&~in[1]&in[0];
assign out[2] = ~in[4]&~in[3]&~in[2]&in[1]&~in[0];
assign out[3] = ~in[4]&~in[3]&~in[2]&in[1]&in[0];
assign out[4] = ~in[4]&~in[3]&in[2]&~in[1]&~in[0];
assign out[5] = ~in[4]&~in[3]&in[2]&~in[1]&in[0];
assign out[6] = ~in[4]&~in[3]&in[2]&in[1]&~in[0];
assign out[7] = ~in[4]&~in[3]&in[2]&in[1]&in[0];
endmodule

module my_and(a,b,out);
	input [31:0] a, b;
	output [31:0] out;

	genvar i;
	generate
		for (i=0;i<32;i=i+1) begin: loop1
			and anded(out[i],a[i],b[i]);
		end
	endgenerate

endmodule

module my_or(a,b,out);
	input [31:0] a, b;
	output [31:0] out;

	genvar i;
	generate
		for (i=0;i<32;i=i+1) begin: loop2
			or ored(out[i],a[i],b[i]);
		end
	endgenerate

endmodule

module my_Lshift(enable, amount, in_data, out_data);
	input enable;
	input [4:0] amount;
	input [31:0] in_data;
	output [31:0] out_data;

	assign out_data = enable ? (in_data << amount) : in_data;
endmodule

module my_Rshift(enable, amount, in_data, out_data);
	input enable;
	input [4:0] amount;
	input signed [31:0] in_data;
	output [31:0] out_data;

	assign out_data = enable ? (in_data >>> amount) : in_data;
endmodule

module my_sll(in,ctrl,out);
	input [4:0] ctrl;
	input [31:0] in;
	output [31:0] out;

	wire [31:0] out16;
	wire [31:0] out8;
	wire [31:0] out4;
	wire [31:0] out2;
	wire [31:0] out1;

	my_Lshift outFor16(ctrl[4],5'b10000,in,out16);
	my_Lshift outFor8(ctrl[3],5'b01000,out16,out8);
	my_Lshift outFor4(ctrl[2],5'b00100,out8,out4);
	my_Lshift outFor2(ctrl[1],5'b00010,out4,out2);
	my_Lshift outFor1(ctrl[0],5'b00001,out2,out1);

	assign out = out1;
endmodule

module my_sra(in,ctrl,out);
	input [4:0] ctrl;
	input [31:0] in;
	output [31:0] out;

	wire [31:0] out16;
	wire [31:0] out8;
	wire [31:0] out4;
	wire [31:0] out2;
	wire [31:0] out1;

	my_Rshift outFor16(ctrl[4],5'b10000,in,out16);
	my_Rshift outFor8(ctrl[3],5'b01000,out16,out8);
	my_Rshift outFor4(ctrl[2],5'b00100,out8,out4);
	my_Rshift outFor2(ctrl[1],5'b00010,out4,out2);
	my_Rshift outFor1(ctrl[0],5'b00001,out2,out1);

	assign out = out1;
endmodule

module my_adderCell(a,b,cin,g,p,s,cout);
	input a, b, cin;
	output g, p, s, cout;

	assign s = a ^ b ^ cin;
	assign g = a & b;
	assign p = a | b;
	assign cout = cin & p | g;

endmodule

module eightBit_Block(a,b,cin,pOut,gOut,s);
	input [7:0] a, b;
	input cin;
	output[7:0] s;
	output pOut;
	output gOut;

	wire [7:0] p,g;
	wire [8:0] c;

	assign c[0] = cin;

	genvar i;
	generate
		for (i=0;i<8;i=i+1) begin: loop3
			my_adderCell adding(a[i],b[i],c[i],g[i],p[i],s[i],c[i+1]);
		end
	endgenerate

	assign pOut = p[0] & p[1] & p[2] & p[3] & p[4] & p[5] & p[6] & p[7];
	assign gOut = g[7] | p[7]&g[6] | p[7]&p[6]&g[5] | p[7]&p[6]&p[5]&g[4] | p[7]&p[6]&p[5]&p[4]&g[3] | p[7]&p[6]&p[5]&p[4]&p[3]&g[2] | p[7]&p[6]&p[5]&p[4]&p[3]&p[2]&g[1] | p[7]&p[6]&p[5]&p[4]&p[3]&p[2]&p[1]&g[0];

endmodule

module my_adder(a,b,c0,s);
	input [31:0] a,b;
	input c0;
	output [31:0] s;

	wire c8,c16,c24,c32;
	wire P0,P1,P2,P3;
	wire G0,G1,G2,G3;

	assign c8 = G0 | P0 & c0;
	assign c16 = G1 | P1 & c8;
	assign c24 = G2 | P2 & c16;
	assign c32 = G3 | P3 & c24;

	eightBit_Block block1(a[7:0],b[7:0],c0,P0,G0,s[7:0]);
	eightBit_Block block2(a[15:8],b[15:8],c8,P1,G1,s[15:8]);
	eightBit_Block block3(a[23:16],b[23:16],c16,P2,G2,s[23:16]);
	eightBit_Block block4(a[31:24],b[31:24],c24,P3,G3,s[31:24]);

endmodule

module my_sub(a,b,cin,d,isNotEqual,isLessThan);
	input [31:0] a,b;
	input cin;
	output [31:0] d;
	output isNotEqual, isLessThan;

	my_adder myadd(a,~b,1'b1,d);

	wire [31:0] out;
	wire [7:0] out2;
	wire [1:0] out3;

	assign out = d;

	assign out2[0] = out[0] | out[1] | out[2] | out[3];
	assign out2[1] = out[4] | out[5] | out[6] | out[7];
	assign out2[2] = out[8] | out[9] | out[10] | out[11];
	assign out2[3] = out[12] | out[13] | out[14] | out[15];
	assign out2[4] = out[16] | out[17] | out[18] | out[19];
	assign out2[5] = out[20] | out[21] | out[22] | out[23];
	assign out2[6] = out[24] | out[25] | out[26] | out[27];
	assign out2[7] = out[28] | out[29] | out[30] | out[31];

	assign out3[0] = out2[0] | out2[1] | out2[2] | out2[3];
	assign out3[1] = out2[4] | out2[5] | out2[6] | out2[7];

	or lastor(isNotEqual,out3[0],out3[1]);

	wire notA, notB;
	wire andNotANotB, andAB, or1and2, or3and1and2;
	wire result1, result2, result3;

	not nota(notA,a[31]);
	not notb(notB,b[31]);

	and and1(andNotANotB,notA,notB);
	and and2(result1,andNotANotB,d[31]);

	and and3(result2,a[31],notB);

	and and4(andAB,a[31],b[31]);
	and and5(result3,andAB,d[31]);

	or or1(or1and2,result1,result2);
	or or2(or3and1and2,result3,or1and2);

	assign isLessThan = or3and1and2;

endmodule


module multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_inputRDY, data_resultRDY);
   input [31:0] data_operandA;
   input [15:0] data_operandB;
   input ctrl_MULT, ctrl_DIV, clock;
   output [31:0] data_result;
   output data_exception, data_inputRDY, data_resultRDY;


  // intermediate products
  wire [31:0] p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15;

  wire [31:0] data_operandA_2sComp;


  assign p0 = data_operandB[0] ? {{16{data_operandA[31]}}, data_operandA} : 32'b0;
  assign p1 = data_operandB[1] ? {{15{data_operandA[31]}}, data_operandA, 1'b0} : 32'b0;
  assign p2 = data_operandB[2] ? {{14{data_operandA[31]}}, data_operandA, 2'b0} : 32'b0;
  assign p3 = data_operandB[3] ? {{13{data_operandA[31]}}, data_operandA, 3'b0} : 32'b0;
  assign p4 = data_operandB[4] ? {{12{data_operandA[31]}}, data_operandA, 4'b0} : 32'b0;
  assign p5 = data_operandB[5] ? {{11{data_operandA[31]}}, data_operandA, 5'b0} : 32'b0;
  assign p6 = data_operandB[6] ? {{10{data_operandA[31]}}, data_operandA, 6'b0} : 32'b0;
  assign p7 = data_operandB[7] ? {{9{data_operandA[31]}}, data_operandA, 7'b0} : 32'b0;
  assign p8 = data_operandB[8] ? {{8{data_operandA[31]}}, data_operandA, 8'b0} : 32'b0;
  assign p9 = data_operandB[9] ? {{7{data_operandA[31]}}, data_operandA, 9'b0} : 32'b0;
  assign p10 = data_operandB[10] ? {{6{data_operandA[31]}}, data_operandA, 10'b0} : 32'b0;
  assign p11 = data_operandB[11] ? {{5{data_operandA[31]}}, data_operandA, 11'b0} : 32'b0;
  assign p12 = data_operandB[12] ? {{4{data_operandA[31]}}, data_operandA, 12'b0} : 32'b0;
  assign p13 = data_operandB[13] ? {{3{data_operandA[31]}}, data_operandA, 13'b0} : 32'b0;
  assign p14 = data_operandB[14] ? {{2{data_operandA[31]}}, data_operandA, 14'b0} : 32'b0;


  assign data_operandA_2sComp = ~data_operandA + 1'b1;
  assign p15 = data_operandB[15] ? {{1{data_operandA_2sComp[31]}},
                                        data_operandA_2sComp, 15'b0} : 32'b0;

  assign data_result = p0 + p1 +
   p2 +p3 +p4+p5 +p6 +p7 + p8 +p9 + p10 + p11 + p12 + p13 + p14 + p15;

    assign data_exception = ~(data_operandB[0] | data_operandB[1] | data_operandB[2] | data_operandB[3] | data_operandB[4] | data_operandB[5]|data_operandB[6]|data_operandB[7] |data_operandB[8]|data_operandB[9]| data_operandB[10] |data_operandB[11]|data_operandB[12]| data_operandB[13]|data_operandB[14]|data_operandB[15]);


endmodule





module divider(data_operandA,data_operandB,data_result, data_exception);

  input [31:0] data_operandA;
  input [31:0] data_operandB;
  output [31:0] data_result;
  output data_exception;
  reg [31:0] data_result = 0;
  reg [31:0] data_A_interm,data_B_interm; // assigned these
  // as registers since Quartus was giving problems in assignments in always block
  reg [32:0] intermed;
    integer i; // used in for loop for division logic

  assign data_exception = (data_operandB == 32'b0) ? 1'b1 : 1'b0;
  always@ (data_operandA or data_operandB)

    begin

        data_A_interm = data_operandA;
        data_B_interm = data_operandB;
        intermed= 0;
      for(i=0;i < 32;i=i+1)    begin
        intermed = {intermed[30:0],data_A_interm[31]};
        data_A_interm[31:1] = data_A_interm[30:0];
        // division logic
        intermed = intermed-data_B_interm;
         data_A_interm[0] = (intermed[31] == 1)?(0):(1);
         intermed = (intermed[31] == 1)?(intermed + data_B_interm):(33'b0);
      end
        data_result = data_A_interm;
// set result to quotient = A
    end


endmodule



/**************************************/

module equal_to_1(regInput, res);
  input [31:0] regInput;
  output res;

  assign res = (regInput == 32'b00000000000000000000000000000001) ? 1'b1 : 1'b0;

endmodule


module equal_to_2(regInput, res);
  input [31:0] regInput;
  output res;

  assign res = (regInput == 32'b00000000000000000000000000000010) ? 1'b1 : 1'b0;

endmodule

/**************************************/

module game_output(board_slot_11, board_slot_12, board_slot_13, board_slot_21, 
board_slot_22, board_slot_23, board_slot_31, board_slot_32, board_slot_33, scoreInput_p1, scoreInput_p2, 
ledArray_red, ledArray_yel, p1_score, p2_score);
  	input [31:0] board_slot_11, board_slot_12, board_slot_13, board_slot_21, board_slot_22, board_slot_23, board_slot_31, board_slot_32, board_slot_33;
  	input [3:0] scoreInput_p1, scoreInput_p2;
  	output [8:0] ledArray_red, ledArray_yel;
		output [6:0] p1_score, p2_score;

	Hexadecimal_To_Seven_Segment hex1(scoreInput_p1, p1_score);
	Hexadecimal_To_Seven_Segment hex2(scoreInput_p2, p2_score);

  equal_to_1 eq11(.regInput(board_slot_11), .res(ledArray_red[0]));
  equal_to_1 eq12(.regInput(board_slot_12), .res(ledArray_red[1]));
  equal_to_1 eq13(.regInput(board_slot_13), .res(ledArray_red[2]));
  equal_to_1 eq14(.regInput(board_slot_21), .res(ledArray_red[3]));
  equal_to_1 eq15(.regInput(board_slot_22), .res(ledArray_red[4]));
  equal_to_1 eq16(.regInput(board_slot_23), .res(ledArray_red[5]));
  equal_to_1 eq17(.regInput(board_slot_31), .res(ledArray_red[6]));
  equal_to_1 eq18(.regInput(board_slot_32), .res(ledArray_red[7]));
  equal_to_1 eq19(.regInput(board_slot_33), .res(ledArray_red[8]));

  equal_to_2 eq21(.regInput(board_slot_11), .res(ledArray_yel[0]));
  equal_to_2 eq22(.regInput(board_slot_12), .res(ledArray_yel[1]));
  equal_to_2 eq23(.regInput(board_slot_13), .res(ledArray_yel[2]));
  equal_to_2 eq24(.regInput(board_slot_21), .res(ledArray_yel[3]));
  equal_to_2 eq25(.regInput(board_slot_22), .res(ledArray_yel[4]));
  equal_to_2 eq26(.regInput(board_slot_23), .res(ledArray_yel[5]));
  equal_to_2 eq27(.regInput(board_slot_31), .res(ledArray_yel[6]));
  equal_to_2 eq28(.regInput(board_slot_32), .res(ledArray_yel[7]));
  equal_to_2 eq29(.regInput(board_slot_33), .res(ledArray_yel[8]));

endmodule

// megafunction wizard: %ALTPLL%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: altpll 

// ============================================================
// File Name: pll.v
// Megafunction Name(s):
// 			altpll
//
// Simulation Library Files(s):
// 			altera_mf
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 9.1 Build 350 03/24/2010 SP 2 SJ Web Edition
// ************************************************************


//Copyright (C) 1991-2010 Altera Corporation
//Your use of Altera Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Altera Program License 
//Subscription Agreement, Altera MegaCore Function License 
//Agreement, or other applicable license agreement, including, 
//without limitation, that your use is for the sole purpose of 
//programming logic devices manufactured by Altera and sold by 
//Altera or its authorized distributors.  Please refer to the 
//applicable agreement for further details.


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module pll (
	inclk0,
	c0);

	input	  inclk0;
	output	  c0;

	wire [5:0] sub_wire0;
	wire [0:0] sub_wire4 = 1'h0;
	wire [0:0] sub_wire1 = sub_wire0[0:0];
	wire  c0 = sub_wire1;
	wire  sub_wire2 = inclk0;
	wire [1:0] sub_wire3 = {sub_wire4, sub_wire2};

	altpll	altpll_component (
				.inclk (sub_wire3),
				.clk (sub_wire0),
				.activeclock (),
				.areset (1'b0),
				.clkbad (),
				.clkena ({6{1'b1}}),
				.clkloss (),
				.clkswitch (1'b0),
				.configupdate (1'b0),
				.enable0 (),
				.enable1 (),
				.extclk (),
				.extclkena ({4{1'b1}}),
				.fbin (1'b1),
				.fbmimicbidir (),
				.fbout (),
				.fref (),
				.icdrclk (),
				.locked (),
				.pfdena (1'b1),
				.phasecounterselect ({4{1'b1}}),
				.phasedone (),
				.phasestep (1'b1),
				.phaseupdown (1'b1),
				.pllena (1'b1),
				.scanaclr (1'b0),
				.scanclk (1'b0),
				.scanclkena (1'b1),
				.scandata (1'b0),
				.scandataout (),
				.scandone (),
				.scanread (1'b0),
				.scanwrite (1'b0),
				.sclkout0 (),
				.sclkout1 (),
				.vcooverrange (),
				.vcounderrange ());
	defparam
		altpll_component.clk0_divide_by = 5,
		altpll_component.clk0_duty_cycle = 50,
		altpll_component.clk0_multiply_by = 2,
		altpll_component.clk0_phase_shift = "0",
		altpll_component.compensate_clock = "CLK0",
		altpll_component.inclk0_input_frequency = 20000,
		//altpll_component.intended_device_family = "Cyclone IV",
		altpll_component.lpm_hint = "CBX_MODULE_PREFIX=pll",
		altpll_component.lpm_type = "altpll",
		altpll_component.operation_mode = "NORMAL",
		altpll_component.port_activeclock = "PORT_UNUSED",
		altpll_component.port_areset = "PORT_UNUSED",
		altpll_component.port_clkbad0 = "PORT_UNUSED",
		altpll_component.port_clkbad1 = "PORT_UNUSED",
		altpll_component.port_clkloss = "PORT_UNUSED",
		altpll_component.port_clkswitch = "PORT_UNUSED",
		altpll_component.port_configupdate = "PORT_UNUSED",
		altpll_component.port_fbin = "PORT_UNUSED",
		altpll_component.port_inclk0 = "PORT_USED",
		altpll_component.port_inclk1 = "PORT_UNUSED",
		altpll_component.port_locked = "PORT_UNUSED",
		altpll_component.port_pfdena = "PORT_UNUSED",
		altpll_component.port_phasecounterselect = "PORT_UNUSED",
		altpll_component.port_phasedone = "PORT_UNUSED",
		altpll_component.port_phasestep = "PORT_UNUSED",
		altpll_component.port_phaseupdown = "PORT_UNUSED",
		altpll_component.port_pllena = "PORT_UNUSED",
		altpll_component.port_scanaclr = "PORT_UNUSED",
		altpll_component.port_scanclk = "PORT_UNUSED",
		altpll_component.port_scanclkena = "PORT_UNUSED",
		altpll_component.port_scandata = "PORT_UNUSED",
		altpll_component.port_scandataout = "PORT_UNUSED",
		altpll_component.port_scandone = "PORT_UNUSED",
		altpll_component.port_scanread = "PORT_UNUSED",
		altpll_component.port_scanwrite = "PORT_UNUSED",
		altpll_component.port_clk0 = "PORT_USED",
		altpll_component.port_clk1 = "PORT_UNUSED",
		altpll_component.port_clk2 = "PORT_UNUSED",
		altpll_component.port_clk3 = "PORT_UNUSED",
		altpll_component.port_clk4 = "PORT_UNUSED",
		altpll_component.port_clk5 = "PORT_UNUSED",
		altpll_component.port_clkena0 = "PORT_UNUSED",
		altpll_component.port_clkena1 = "PORT_UNUSED",
		altpll_component.port_clkena2 = "PORT_UNUSED",
		altpll_component.port_clkena3 = "PORT_UNUSED",
		altpll_component.port_clkena4 = "PORT_UNUSED",
		altpll_component.port_clkena5 = "PORT_UNUSED",
		altpll_component.port_extclk0 = "PORT_UNUSED",
		altpll_component.port_extclk1 = "PORT_UNUSED",
		altpll_component.port_extclk2 = "PORT_UNUSED",
		altpll_component.port_extclk3 = "PORT_UNUSED";


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ACTIVECLK_CHECK STRING "0"
// Retrieval info: PRIVATE: BANDWIDTH STRING "1.000"
// Retrieval info: PRIVATE: BANDWIDTH_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: BANDWIDTH_FREQ_UNIT STRING "MHz"
// Retrieval info: PRIVATE: BANDWIDTH_PRESET STRING "Low"
// Retrieval info: PRIVATE: BANDWIDTH_USE_AUTO STRING "1"
// Retrieval info: PRIVATE: BANDWIDTH_USE_CUSTOM STRING "0"
// Retrieval info: PRIVATE: BANDWIDTH_USE_PRESET STRING "0"
// Retrieval info: PRIVATE: CLKBAD_SWITCHOVER_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKLOSS_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKSWITCH_CHECK STRING "1"
// Retrieval info: PRIVATE: CNX_NO_COMPENSATE_RADIO STRING "0"
// Retrieval info: PRIVATE: CREATE_CLKBAD_CHECK STRING "0"
// Retrieval info: PRIVATE: CREATE_INCLK1_CHECK STRING "0"
// Retrieval info: PRIVATE: CUR_DEDICATED_CLK STRING "c0"
// Retrieval info: PRIVATE: CUR_FBIN_CLK STRING "e0"
// Retrieval info: PRIVATE: DEVICE_SPEED_GRADE STRING "6"
// Retrieval info: PRIVATE: DIV_FACTOR0 NUMERIC "1"
// Retrieval info: PRIVATE: DUTY_CYCLE0 STRING "50.00000000"
// Retrieval info: PRIVATE: EFF_OUTPUT_FREQ_VALUE0 STRING "20.000000"
// Retrieval info: PRIVATE: EXPLICIT_SWITCHOVER_COUNTER STRING "0"
// Retrieval info: PRIVATE: EXT_FEEDBACK_RADIO STRING "0"
// Retrieval info: PRIVATE: GLOCKED_COUNTER_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: GLOCKED_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: GLOCKED_MODE_CHECK STRING "0"
// Retrieval info: PRIVATE: GLOCK_COUNTER_EDIT NUMERIC "1048575"
// Retrieval info: PRIVATE: HAS_MANUAL_SWITCHOVER STRING "1"
// Retrieval info: PRIVATE: INCLK0_FREQ_EDIT STRING "50.000"
// Retrieval info: PRIVATE: INCLK0_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT STRING "100.000"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone II"
// Retrieval info: PRIVATE: INT_FEEDBACK__MODE_RADIO STRING "1"
// Retrieval info: PRIVATE: LOCKED_OUTPUT_CHECK STRING "0"
// Retrieval info: PRIVATE: LONG_SCAN_RADIO STRING "1"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE STRING "300.000"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE_DIRTY NUMERIC "0"
// Retrieval info: PRIVATE: LVDS_PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: MIG_DEVICE_SPEED_GRADE STRING "Any"
// Retrieval info: PRIVATE: MIRROR_CLK0 STRING "0"
// Retrieval info: PRIVATE: MULT_FACTOR0 NUMERIC "1"
// Retrieval info: PRIVATE: NORMAL_MODE_RADIO STRING "1"
// Retrieval info: PRIVATE: OUTPUT_FREQ0 STRING "20.00000000"
// Retrieval info: PRIVATE: OUTPUT_FREQ_MODE0 STRING "1"
// Retrieval info: PRIVATE: OUTPUT_FREQ_UNIT0 STRING "MHz"
// Retrieval info: PRIVATE: PHASE_RECONFIG_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: PHASE_RECONFIG_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT0 STRING "0.00000000"
// Retrieval info: PRIVATE: PHASE_SHIFT_STEP_ENABLED_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: PLL_ADVANCED_PARAM_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_ARESET_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_AUTOPLL_CHECK NUMERIC "1"
// Retrieval info: PRIVATE: PLL_ENA_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_ENHPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FASTPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FBMIMIC_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_LVDS_PLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_PFDENA_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_TARGET_HARCOPY_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PRIMARY_CLK_COMBO STRING "inclk0"
// Retrieval info: PRIVATE: RECONFIG_FILE STRING "pll.mif"
// Retrieval info: PRIVATE: SACN_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: SCAN_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: SELF_RESET_LOCK_LOSS STRING "0"
// Retrieval info: PRIVATE: SHORT_SCAN_RADIO STRING "0"
// Retrieval info: PRIVATE: SPREAD_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: SPREAD_FREQ STRING "50.000"
// Retrieval info: PRIVATE: SPREAD_FREQ_UNIT STRING "KHz"
// Retrieval info: PRIVATE: SPREAD_PERCENT STRING "0.500"
// Retrieval info: PRIVATE: SPREAD_USE STRING "0"
// Retrieval info: PRIVATE: SRC_SYNCH_COMP_RADIO STRING "0"
// Retrieval info: PRIVATE: STICKY_CLK0 STRING "1"
// Retrieval info: PRIVATE: SWITCHOVER_COUNT_EDIT NUMERIC "1"
// Retrieval info: PRIVATE: SWITCHOVER_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: USE_CLK0 STRING "1"
// Retrieval info: PRIVATE: USE_CLKENA0 STRING "0"
// Retrieval info: PRIVATE: USE_MIL_SPEED_GRADE NUMERIC "0"
// Retrieval info: PRIVATE: ZERO_DELAY_RADIO STRING "0"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: CLK0_DIVIDE_BY NUMERIC "5"
// Retrieval info: CONSTANT: CLK0_DUTY_CYCLE NUMERIC "50"
// Retrieval info: CONSTANT: CLK0_MULTIPLY_BY NUMERIC "2"
// Retrieval info: CONSTANT: CLK0_PHASE_SHIFT STRING "0"
// Retrieval info: CONSTANT: COMPENSATE_CLOCK STRING "CLK0"
// Retrieval info: CONSTANT: INCLK0_INPUT_FREQUENCY NUMERIC "20000"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone II"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altpll"
// Retrieval info: CONSTANT: OPERATION_MODE STRING "NORMAL"
// Retrieval info: CONSTANT: PORT_ACTIVECLOCK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_ARESET STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKLOSS STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKSWITCH STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CONFIGUPDATE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_FBIN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_INCLK0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_INCLK1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_LOCKED STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PFDENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASECOUNTERSELECT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASESTEP STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEUPDOWN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PLLENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANACLR STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLKENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATAOUT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANREAD STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANWRITE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_clk1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk3 STRING "PORT_UNUSED"
// Retrieval info: USED_PORT: @clk 0 0 6 0 OUTPUT_CLK_EXT VCC "@clk[5..0]"
// Retrieval info: USED_PORT: @extclk 0 0 4 0 OUTPUT_CLK_EXT VCC "@extclk[3..0]"
// Retrieval info: USED_PORT: c0 0 0 0 0 OUTPUT_CLK_EXT VCC "c0"
// Retrieval info: USED_PORT: inclk0 0 0 0 0 INPUT_CLK_EXT GND "inclk0"
// Retrieval info: CONNECT: @inclk 0 0 1 0 inclk0 0 0 0 0
// Retrieval info: CONNECT: c0 0 0 0 0 @clk 0 0 1 0
// Retrieval info: CONNECT: @inclk 0 0 1 1 GND 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL pll.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL pll.ppf TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL pll.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL pll.cmp TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL pll.bsf TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL pll_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL pll_bb.v FALSE
// Retrieval info: LIB_FILE: altera_mf
// Retrieval info: CBX_MODULE_PREFIX: ON

/*****************************************************************************
 *                                                                           *
 * Module:       Altera_UP_PS2_Data_In                                       *
 * Description:                                                              *
 *      This module accepts incoming data from a PS2 core.                   *
 *                                                                           *
 *****************************************************************************/


module Altera_UP_PS2_Data_In (
	// Inputs
	clk,
	reset,

	wait_for_incoming_data,
	start_receiving_data,

	ps2_clk_posedge,
	ps2_clk_negedge,
	ps2_data,

	// Bidirectionals

	// Outputs
	received_data,
	received_data_en			// If 1 - new data has been received
);


/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input				wait_for_incoming_data;
input				start_receiving_data;

input				ps2_clk_posedge;
input				ps2_clk_negedge;
input			 	ps2_data;

// Bidirectionals

// Outputs
output reg	[7:0]	received_data;

output reg		 	received_data_en;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
// states
localparam	PS2_STATE_0_IDLE			= 3'h0,
			PS2_STATE_1_WAIT_FOR_DATA	= 3'h1,
			PS2_STATE_2_DATA_IN			= 3'h2,
			PS2_STATE_3_PARITY_IN		= 3'h3,
			PS2_STATE_4_STOP_IN			= 3'h4;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
reg			[3:0]	data_count;
reg			[7:0]	data_shift_reg;

// State Machine Registers
reg			[2:0]	ns_ps2_receiver;
reg			[2:0]	s_ps2_receiver;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		s_ps2_receiver <= PS2_STATE_0_IDLE;
	else
		s_ps2_receiver <= ns_ps2_receiver;
end

always @(*)
begin
	// Defaults
	ns_ps2_receiver = PS2_STATE_0_IDLE;

    case (s_ps2_receiver)
	PS2_STATE_0_IDLE:
		begin
			if ((wait_for_incoming_data == 1'b1) && 
					(received_data_en == 1'b0))
				ns_ps2_receiver = PS2_STATE_1_WAIT_FOR_DATA;
			else if ((start_receiving_data == 1'b1) && 
					(received_data_en == 1'b0))
				ns_ps2_receiver = PS2_STATE_2_DATA_IN;
			else
				ns_ps2_receiver = PS2_STATE_0_IDLE;
		end
	PS2_STATE_1_WAIT_FOR_DATA:
		begin
			if ((ps2_data == 1'b0) && (ps2_clk_posedge == 1'b1))
				ns_ps2_receiver = PS2_STATE_2_DATA_IN;
			else if (wait_for_incoming_data == 1'b0)
				ns_ps2_receiver = PS2_STATE_0_IDLE;
			else
				ns_ps2_receiver = PS2_STATE_1_WAIT_FOR_DATA;
		end
	PS2_STATE_2_DATA_IN:
		begin
			if ((data_count == 3'h7) && (ps2_clk_posedge == 1'b1))
				ns_ps2_receiver = PS2_STATE_3_PARITY_IN;
			else
				ns_ps2_receiver = PS2_STATE_2_DATA_IN;
		end
	PS2_STATE_3_PARITY_IN:
		begin
			if (ps2_clk_posedge == 1'b1)
				ns_ps2_receiver = PS2_STATE_4_STOP_IN;
			else
				ns_ps2_receiver = PS2_STATE_3_PARITY_IN;
		end
	PS2_STATE_4_STOP_IN:
		begin
			if (ps2_clk_posedge == 1'b1)
				ns_ps2_receiver = PS2_STATE_0_IDLE;
			else
				ns_ps2_receiver = PS2_STATE_4_STOP_IN;
		end
	default:
		begin
			ns_ps2_receiver = PS2_STATE_0_IDLE;
		end
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/


always @(posedge clk)
begin
	if (reset == 1'b1) 
		data_count	<= 3'h0;
	else if ((s_ps2_receiver == PS2_STATE_2_DATA_IN) && 
			(ps2_clk_posedge == 1'b1))
		data_count	<= data_count + 3'h1;
	else if (s_ps2_receiver != PS2_STATE_2_DATA_IN)
		data_count	<= 3'h0;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		data_shift_reg			<= 8'h00;
	else if ((s_ps2_receiver == PS2_STATE_2_DATA_IN) && 
			(ps2_clk_posedge == 1'b1))
		data_shift_reg	<= {ps2_data, data_shift_reg[7:1]};
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		received_data		<= 8'h00;
	else if (s_ps2_receiver == PS2_STATE_4_STOP_IN)
		received_data	<= data_shift_reg;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		received_data_en		<= 1'b0;
	else if ((s_ps2_receiver == PS2_STATE_4_STOP_IN) &&
			(ps2_clk_posedge == 1'b1))
		received_data_en	<= 1'b1;
	else
		received_data_en	<= 1'b0;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


endmodule

/*****************************************************************************
 *                                                                           *
 * Module:       Altera_UP_PS2_Command_Out                                   *
 * Description:                                                              *
 *      This module sends commands out to the PS2 core.                      *
 *                                                                           *
 *****************************************************************************/


module Altera_UP_PS2_Command_Out (
	// Inputs
	clk,
	reset,

	the_command,
	send_command,

	ps2_clk_posedge,
	ps2_clk_negedge,

	// Bidirectionals
	PS2_CLK,
 	PS2_DAT,

	// Outputs
	command_was_sent,
	error_communication_timed_out
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

// Timing info for initiating Host-to-Device communication 
//   when using a 50MHz system clock
parameter	CLOCK_CYCLES_FOR_101US		= 5050;
parameter	NUMBER_OF_BITS_FOR_101US	= 13;
parameter	COUNTER_INCREMENT_FOR_101US	= 13'h0001;

//parameter	CLOCK_CYCLES_FOR_101US		= 50;
//parameter	NUMBER_OF_BITS_FOR_101US	= 6;
//parameter	COUNTER_INCREMENT_FOR_101US	= 6'h01;

// Timing info for start of transmission error 
//   when using a 50MHz system clock
parameter	CLOCK_CYCLES_FOR_15MS		= 750000;
parameter	NUMBER_OF_BITS_FOR_15MS		= 20;
parameter	COUNTER_INCREMENT_FOR_15MS	= 20'h00001;

// Timing info for sending data error 
//   when using a 50MHz system clock
parameter	CLOCK_CYCLES_FOR_2MS		= 100000;
parameter	NUMBER_OF_BITS_FOR_2MS		= 17;
parameter	COUNTER_INCREMENT_FOR_2MS	= 17'h00001;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				clk;
input				reset;

input		[7:0]	the_command;
input				send_command;

input				ps2_clk_posedge;
input				ps2_clk_negedge;

// Bidirectionals
inout				PS2_CLK;
inout			 	PS2_DAT;

// Outputs
output	reg			command_was_sent;
output	reg		 	error_communication_timed_out;

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
// states
parameter	PS2_STATE_0_IDLE					= 3'h0,
			PS2_STATE_1_INITIATE_COMMUNICATION	= 3'h1,
			PS2_STATE_2_WAIT_FOR_CLOCK			= 3'h2,
			PS2_STATE_3_TRANSMIT_DATA			= 3'h3,
			PS2_STATE_4_TRANSMIT_STOP_BIT		= 3'h4,
			PS2_STATE_5_RECEIVE_ACK_BIT			= 3'h5,
			PS2_STATE_6_COMMAND_WAS_SENT		= 3'h6,
			PS2_STATE_7_TRANSMISSION_ERROR		= 3'h7;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires

// Internal Registers
reg			[3:0]	cur_bit;
reg			[8:0]	ps2_command;

reg			[NUMBER_OF_BITS_FOR_101US:1]	command_initiate_counter;

reg			[NUMBER_OF_BITS_FOR_15MS:1]		waiting_counter;
reg			[NUMBER_OF_BITS_FOR_2MS:1]		transfer_counter;

// State Machine Registers
reg			[2:0]	ns_ps2_transmitter;
reg			[2:0]	s_ps2_transmitter;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		s_ps2_transmitter <= PS2_STATE_0_IDLE;
	else
		s_ps2_transmitter <= ns_ps2_transmitter;
end

always @(*)
begin
	// Defaults
	ns_ps2_transmitter = PS2_STATE_0_IDLE;

    case (s_ps2_transmitter)
	PS2_STATE_0_IDLE:
		begin
			if (send_command == 1'b1)
				ns_ps2_transmitter = PS2_STATE_1_INITIATE_COMMUNICATION;
			else
				ns_ps2_transmitter = PS2_STATE_0_IDLE;
		end
	PS2_STATE_1_INITIATE_COMMUNICATION:
		begin
			if (command_initiate_counter == CLOCK_CYCLES_FOR_101US)
				ns_ps2_transmitter = PS2_STATE_2_WAIT_FOR_CLOCK;
			else
				ns_ps2_transmitter = PS2_STATE_1_INITIATE_COMMUNICATION;
		end
	PS2_STATE_2_WAIT_FOR_CLOCK:
		begin
			if (ps2_clk_negedge == 1'b1)
				ns_ps2_transmitter = PS2_STATE_3_TRANSMIT_DATA;
			else if (waiting_counter == CLOCK_CYCLES_FOR_15MS)
				ns_ps2_transmitter = PS2_STATE_7_TRANSMISSION_ERROR;
			else
				ns_ps2_transmitter = PS2_STATE_2_WAIT_FOR_CLOCK;
		end
	PS2_STATE_3_TRANSMIT_DATA:
		begin
			if ((cur_bit == 4'd8) && (ps2_clk_negedge == 1'b1))
				ns_ps2_transmitter = PS2_STATE_4_TRANSMIT_STOP_BIT;
			else if (transfer_counter == CLOCK_CYCLES_FOR_2MS)
				ns_ps2_transmitter = PS2_STATE_7_TRANSMISSION_ERROR;
			else
				ns_ps2_transmitter = PS2_STATE_3_TRANSMIT_DATA;
		end
	PS2_STATE_4_TRANSMIT_STOP_BIT:
		begin
			if (ps2_clk_negedge == 1'b1)
				ns_ps2_transmitter = PS2_STATE_5_RECEIVE_ACK_BIT;
			else if (transfer_counter == CLOCK_CYCLES_FOR_2MS)
				ns_ps2_transmitter = PS2_STATE_7_TRANSMISSION_ERROR;
			else
				ns_ps2_transmitter = PS2_STATE_4_TRANSMIT_STOP_BIT;
		end
	PS2_STATE_5_RECEIVE_ACK_BIT:
		begin
			if (ps2_clk_posedge == 1'b1)
				ns_ps2_transmitter = PS2_STATE_6_COMMAND_WAS_SENT;
			else if (transfer_counter == CLOCK_CYCLES_FOR_2MS)
				ns_ps2_transmitter = PS2_STATE_7_TRANSMISSION_ERROR;
			else
				ns_ps2_transmitter = PS2_STATE_5_RECEIVE_ACK_BIT;
		end
	PS2_STATE_6_COMMAND_WAS_SENT:
		begin
			if (send_command == 1'b0)
				ns_ps2_transmitter = PS2_STATE_0_IDLE;
			else
				ns_ps2_transmitter = PS2_STATE_6_COMMAND_WAS_SENT;
		end
	PS2_STATE_7_TRANSMISSION_ERROR:
		begin
			if (send_command == 1'b0)
				ns_ps2_transmitter = PS2_STATE_0_IDLE;
			else
				ns_ps2_transmitter = PS2_STATE_7_TRANSMISSION_ERROR;
		end
	default:
		begin
			ns_ps2_transmitter = PS2_STATE_0_IDLE;
		end
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge clk)
begin
	if (reset == 1'b1)
		ps2_command <= 9'h000;
	else if (s_ps2_transmitter == PS2_STATE_0_IDLE)
		ps2_command <= {(^the_command) ^ 1'b1, the_command};
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		command_initiate_counter <= {NUMBER_OF_BITS_FOR_101US{1'b0}};
	else if ((s_ps2_transmitter == PS2_STATE_1_INITIATE_COMMUNICATION) &&
			(command_initiate_counter != CLOCK_CYCLES_FOR_101US))
		command_initiate_counter <= 
			command_initiate_counter + COUNTER_INCREMENT_FOR_101US;
	else if (s_ps2_transmitter != PS2_STATE_1_INITIATE_COMMUNICATION)
		command_initiate_counter <= {NUMBER_OF_BITS_FOR_101US{1'b0}};
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		waiting_counter <= {NUMBER_OF_BITS_FOR_15MS{1'b0}};
	else if ((s_ps2_transmitter == PS2_STATE_2_WAIT_FOR_CLOCK) &&
			(waiting_counter != CLOCK_CYCLES_FOR_15MS))
		waiting_counter <= waiting_counter + COUNTER_INCREMENT_FOR_15MS;
	else if (s_ps2_transmitter != PS2_STATE_2_WAIT_FOR_CLOCK)
		waiting_counter <= {NUMBER_OF_BITS_FOR_15MS{1'b0}};
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		transfer_counter <= {NUMBER_OF_BITS_FOR_2MS{1'b0}};
	else
	begin
		if ((s_ps2_transmitter == PS2_STATE_3_TRANSMIT_DATA) ||
			(s_ps2_transmitter == PS2_STATE_4_TRANSMIT_STOP_BIT) ||
			(s_ps2_transmitter == PS2_STATE_5_RECEIVE_ACK_BIT))
		begin
			if (transfer_counter != CLOCK_CYCLES_FOR_2MS)
				transfer_counter <= transfer_counter + COUNTER_INCREMENT_FOR_2MS;
		end
		else
			transfer_counter <= {NUMBER_OF_BITS_FOR_2MS{1'b0}};
	end
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		cur_bit <= 4'h0;
	else if ((s_ps2_transmitter == PS2_STATE_3_TRANSMIT_DATA) &&
			(ps2_clk_negedge == 1'b1))
		cur_bit <= cur_bit + 4'h1;
	else if (s_ps2_transmitter != PS2_STATE_3_TRANSMIT_DATA)
		cur_bit <= 4'h0;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		command_was_sent <= 1'b0;
	else if (s_ps2_transmitter == PS2_STATE_6_COMMAND_WAS_SENT)
		command_was_sent <= 1'b1;
	else if (send_command == 1'b0)
			command_was_sent <= 1'b0;
end

always @(posedge clk)
begin
	if (reset == 1'b1)
		error_communication_timed_out <= 1'b0;
	else if (s_ps2_transmitter == PS2_STATE_7_TRANSMISSION_ERROR)
		error_communication_timed_out <= 1'b1;
	else if (send_command == 1'b0)
		error_communication_timed_out <= 1'b0;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign PS2_CLK	= 
	(s_ps2_transmitter == PS2_STATE_1_INITIATE_COMMUNICATION) ? 
		1'b0 :
		1'bz;

assign PS2_DAT	= 
	(s_ps2_transmitter == PS2_STATE_3_TRANSMIT_DATA) ? ps2_command[cur_bit] :
	(s_ps2_transmitter == PS2_STATE_2_WAIT_FOR_CLOCK) ? 1'b0 :
	((s_ps2_transmitter == PS2_STATE_1_INITIATE_COMMUNICATION) && 
		(command_initiate_counter[NUMBER_OF_BITS_FOR_101US] == 1'b1)) ? 1'b0 : 
			1'bz;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


endmodule

module PS2_Interface(inclock, resetn, ps2_clock, ps2_data, ps2_key_data, ps2_key_pressed, last_data_received);

	input 			inclock, resetn;
	inout 			ps2_clock, ps2_data;
	output 			ps2_key_pressed;
	output 	[7:0] 	ps2_key_data;
	output 	[7:0] 	last_data_received;

	// Internal Registers
	reg			[7:0]	last_data_received;	
	
	always @(posedge inclock)
	begin
		if (resetn == 1'b0)
			last_data_received <= 8'h00;
		else if (ps2_key_pressed == 1'b1)
			last_data_received <= ps2_key_data;
	end
	
	PS2_Controller PS2 (.CLOCK_50 			(inclock),
						.reset 				(~resetn),
						.PS2_CLK			(ps2_clock),
						.PS2_DAT			(ps2_data),		
						.received_data		(ps2_key_data),
						.received_data_en	(ps2_key_pressed)
						);

endmodule

/*****************************************************************************
 *                                                                           *
 * Module:       Altera_UP_PS2                                               *
 * Description:                                                              *
 *      This module communicates with the PS2 core.                          *
 *                                                                           *
 *****************************************************************************/

module PS2_Controller #(parameter INITIALIZE_MOUSE = 0) (
	// Inputs
	CLOCK_50,
	reset,

	the_command,
	send_command,

	// Bidirectionals
	PS2_CLK,					// PS2 Clock
 	PS2_DAT,					// PS2 Data

	// Outputs
	command_was_sent,
	error_communication_timed_out,

	received_data,
	received_data_en			// If 1 - new data has been received
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			CLOCK_50;
input			reset;

input	[7:0]	the_command;
input			send_command;

// Bidirectionals
inout			PS2_CLK;
inout		 	PS2_DAT;

// Outputs
output			command_was_sent;
output			error_communication_timed_out;

output	[7:0]	received_data;
output		 	received_data_en;

wire [7:0] the_command_w;
wire send_command_w, command_was_sent_w, error_communication_timed_out_w;

generate
	if(INITIALIZE_MOUSE) begin
  		reg init_done;
	  		
		assign the_command_w = init_done ? the_command : 8'hf4;
		assign send_command_w = init_done ? send_command : (!command_was_sent_w && !error_communication_timed_out_w);
		assign command_was_sent = init_done ? command_was_sent_w : 0;
		assign error_communication_timed_out = init_done ? error_communication_timed_out_w : 1;
		

		
		always @(posedge CLOCK_50)
			if(reset) init_done <= 0;
			else if(command_was_sent_w) init_done <= 1;
		
	end else begin
		assign the_command_w = the_command;
		assign send_command_w = send_command;
		assign command_was_sent = command_was_sent_w;
		assign error_communication_timed_out = error_communication_timed_out_w;
	end
endgenerate

/*****************************************************************************
 *                           Constant Declarations                           *
 *****************************************************************************/
// states
localparam	PS2_STATE_0_IDLE			= 3'h0,
			PS2_STATE_1_DATA_IN			= 3'h1,
			PS2_STATE_2_COMMAND_OUT		= 3'h2,
			PS2_STATE_3_END_TRANSFER	= 3'h3,
			PS2_STATE_4_END_DELAYED		= 3'h4;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			ps2_clk_posedge;
wire			ps2_clk_negedge;

wire			start_receiving_data;
wire			wait_for_incoming_data;

// Internal Registers
reg		[7:0]	idle_counter;

reg				ps2_clk_reg;
reg				ps2_data_reg;
reg				last_ps2_clk;

// State Machine Registers
reg		[2:0]	ns_ps2_transceiver;
reg		[2:0]	s_ps2_transceiver;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge CLOCK_50)
begin
	if (reset == 1'b1)
		s_ps2_transceiver <= PS2_STATE_0_IDLE;
	else
		s_ps2_transceiver <= ns_ps2_transceiver;
end

always @(*)
begin
	// Defaults
	ns_ps2_transceiver = PS2_STATE_0_IDLE;

    case (s_ps2_transceiver)
	PS2_STATE_0_IDLE:
		begin
			if ((idle_counter == 8'hFF) && 
					(send_command == 1'b1))
				ns_ps2_transceiver = PS2_STATE_2_COMMAND_OUT;
			else if ((ps2_data_reg == 1'b0) && (ps2_clk_posedge == 1'b1))
				ns_ps2_transceiver = PS2_STATE_1_DATA_IN;
			else
				ns_ps2_transceiver = PS2_STATE_0_IDLE;
		end
	PS2_STATE_1_DATA_IN:
		begin
			if ((received_data_en == 1'b1)/* && (ps2_clk_posedge == 1'b1)*/)
				ns_ps2_transceiver = PS2_STATE_0_IDLE;
			else
				ns_ps2_transceiver = PS2_STATE_1_DATA_IN;
		end
	PS2_STATE_2_COMMAND_OUT:
		begin
			if ((command_was_sent == 1'b1) ||
				(error_communication_timed_out == 1'b1))
				ns_ps2_transceiver = PS2_STATE_3_END_TRANSFER;
			else
				ns_ps2_transceiver = PS2_STATE_2_COMMAND_OUT;
		end
	PS2_STATE_3_END_TRANSFER:
		begin
			if (send_command == 1'b0)
				ns_ps2_transceiver = PS2_STATE_0_IDLE;
			else if ((ps2_data_reg == 1'b0) && (ps2_clk_posedge == 1'b1))
				ns_ps2_transceiver = PS2_STATE_4_END_DELAYED;
			else
				ns_ps2_transceiver = PS2_STATE_3_END_TRANSFER;
		end
	PS2_STATE_4_END_DELAYED:	
		begin
			if (received_data_en == 1'b1)
			begin
				if (send_command == 1'b0)
					ns_ps2_transceiver = PS2_STATE_0_IDLE;
				else
					ns_ps2_transceiver = PS2_STATE_3_END_TRANSFER;
			end
			else
				ns_ps2_transceiver = PS2_STATE_4_END_DELAYED;
		end	
	default:
			ns_ps2_transceiver = PS2_STATE_0_IDLE;
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
begin
	if (reset == 1'b1)
	begin
		last_ps2_clk	<= 1'b1;
		ps2_clk_reg		<= 1'b1;

		ps2_data_reg	<= 1'b1;
	end
	else
	begin
		last_ps2_clk	<= ps2_clk_reg;
		ps2_clk_reg		<= PS2_CLK;

		ps2_data_reg	<= PS2_DAT;
	end
end

always @(posedge CLOCK_50)
begin
	if (reset == 1'b1)
		idle_counter <= 6'h00;
	else if ((s_ps2_transceiver == PS2_STATE_0_IDLE) &&
			(idle_counter != 8'hFF))
		idle_counter <= idle_counter + 6'h01;
	else if (s_ps2_transceiver != PS2_STATE_0_IDLE)
		idle_counter <= 6'h00;
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign ps2_clk_posedge = 
			((ps2_clk_reg == 1'b1) && (last_ps2_clk == 1'b0)) ? 1'b1 : 1'b0;
assign ps2_clk_negedge = 
			((ps2_clk_reg == 1'b0) && (last_ps2_clk == 1'b1)) ? 1'b1 : 1'b0;

assign start_receiving_data		= (s_ps2_transceiver == PS2_STATE_1_DATA_IN);
assign wait_for_incoming_data	= 
			(s_ps2_transceiver == PS2_STATE_3_END_TRANSFER);

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Altera_UP_PS2_Data_In PS2_Data_In (
	// Inputs
	.clk							(CLOCK_50),
	.reset							(reset),

	.wait_for_incoming_data			(wait_for_incoming_data),
	.start_receiving_data			(start_receiving_data),

	.ps2_clk_posedge				(ps2_clk_posedge),
	.ps2_clk_negedge				(ps2_clk_negedge),
	.ps2_data						(ps2_data_reg),

	// Bidirectionals

	// Outputs
	.received_data					(received_data),
	.received_data_en				(received_data_en)
);

Altera_UP_PS2_Command_Out PS2_Command_Out (
	// Inputs
	.clk							(CLOCK_50),
	.reset							(reset),

	.the_command					(the_command_w),
	.send_command					(send_command_w),

	.ps2_clk_posedge				(ps2_clk_posedge),
	.ps2_clk_negedge				(ps2_clk_negedge),

	// Bidirectionals
	.PS2_CLK						(PS2_CLK),
 	.PS2_DAT						(PS2_DAT),

	// Outputs
	.command_was_sent				(command_was_sent_w),
	.error_communication_timed_out	(error_communication_timed_out_w)
);

endmodule

/******************************************************************************
 *                                                                            *
 * Module:       Hexadecimal_To_Seven_Segment                                 *
 * Description:                                                               *
 *      This module converts hexadecimal numbers for seven segment displays.  *
 *                                                                            *
 ******************************************************************************/

module Hexadecimal_To_Seven_Segment (
	// Inputs
	hex_number,

	// Bidirectional

	// Outputs
	seven_seg_display
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input		[3:0]	hex_number;

// Bidirectional

// Outputs
output		[6:0]	seven_seg_display;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/


/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

assign seven_seg_display =
		({7{(hex_number == 4'h0)}} & 7'b1000000) |
		({7{(hex_number == 4'h1)}} & 7'b1111001) |
		({7{(hex_number == 4'h2)}} & 7'b0100100) |
		({7{(hex_number == 4'h3)}} & 7'b0110000) |
		({7{(hex_number == 4'h4)}} & 7'b0011001) |
		({7{(hex_number == 4'h5)}} & 7'b0010010) |
		({7{(hex_number == 4'h6)}} & 7'b0000010) |
		({7{(hex_number == 4'h7)}} & 7'b1111000) |
		({7{(hex_number == 4'h8)}} & 7'b0000000) |
		({7{(hex_number == 4'h9)}} & 7'b0010000) |
		({7{(hex_number == 4'hA)}} & 7'b0001000) |
		({7{(hex_number == 4'hB)}} & 7'b0000011) |
		({7{(hex_number == 4'hC)}} & 7'b1000110) |
		({7{(hex_number == 4'hD)}} & 7'b0100001) |
		({7{(hex_number == 4'hE)}} & 7'b0000110) |
		({7{(hex_number == 4'hF)}} & 7'b0001110); 

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/


endmodule

