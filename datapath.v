`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/14/2024 05:07:58 PM
// Design Name: 
// Module Name: datapath
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module datapath(
    input clk
    );
	//WIRES
		//IF 
			wire [31:0] pc, nextPc, inst;
		//ID 
			wire [0:0] regWrite, memToReg, memWrite, aluSrc, regDst, memRead, stall;
    		wire [3:0] aluControl;
			wire [31:0] inst_d, imm32, regOut1, regOut2;
			reg [4:0] rs, rt, rd;
    		reg [5:0] opcode, func;
			reg [15:0] imm16;
			always @(*) begin
      		 	rs = inst_d[25:21];
      		 	rt = inst_d[20:16];
      		 	rd = inst_d[15:11];
				opcode = inst_d[31:26];
       			func = inst_d[5:0];
				imm16 = inst_d[15:0];
   			end
		//EX
			wire [0:0] regWrite_x, memToReg_x, memWrite_x, aluSrc_x, regDst_x, memRead_x;
    		wire [3:0] aluControl_x;
			wire [31:0] regOut1_x, regOut2_x, imm32_x, aluOut, aluIn2, aluIn1, muxIn1;
			wire [4:0] writeAddr, rt_x, rd_x, rs_x;
			wire [1:0] forwardA, forwardB;
		//MEM
			wire regWrite_m, memToReg_m, memRead_m, memWrite_m;
			wire [31:0] aluOut_m, regOut2_m, memOut;
			wire [4:0] writeAddr_m;
		//WB
			wire regWrite_b, memToReg_b;
			wire [31:0] aluOut_b, memOut_b, writeData;
			wire [4:0] writeAddr_b;




	//MODULES
		//IF
			program_counter pcmodule(.clk(clk), .nextPc(nextPc), .pc(pc), .stall(stall));
    		pc_adder pa(.pc(pc), .offset(32'd4), .nextPc(nextPc));
    		inst_mem im(.pc(pc), .inst(inst));


		IFID IFIDreg(.clk(clk), .inst(inst), .inst_d(inst_d), .stall(stall));


    	//ID
			register_file regFile(.readAddr1(rs), .readAddr2(rt), .regOut1(regOut1), .regOut2(regOut2), .regWrite(regWrite_b), .clk(clk), .writeData(writeData), .writeAddr(writeAddr_b));
			control_unit CU(.op(opcode), .func(func), .regWrite(regWrite), .memToReg(memToReg), .memWrite(memWrite), .aluSrc(aluSrc), .regDst(regDst), .memRead(memRead), .aluControl(aluControl), .stall(stall));
			imm_extend immex(.imm(imm16), .imm32(imm32));
            hazard_unit hu(.memRead_x(memRead_x), .rt_x(rt_x), .rt(rt), .rs(rs), .stall(stall));
            
            

		IDEX IDEXreg(.clk(clk), .regWrite(regWrite), .memToReg(memToReg), .memWrite(memWrite), .aluSrc(aluSrc), .regDst(regDst), .memRead(memRead), .aluControl(aluControl), .regOut1(regOut1), .regOut2(regOut2), .imm32(imm32), .rt(rt), .rd(rd), .rs(rs), .regWrite_x(regWrite_x), .memToReg_x(memToReg_x), .memWrite_x(memWrite_x), .aluSrc_x(aluSrc_x), .regDst_x(regDst_x), .memRead_x(memRead_x), .aluControl_x(aluControl_x), .regOut1_x(regOut1_x), .regOut2_x(regOut2_x), .imm32_x(imm32_x), .rt_x(rt_x), .rd_x(rd_x), .rs_x(rs_x));
		

		//EX
			alu ALU(.aluIn1(aluIn1), .aluIn2(aluIn2), .aluControl(aluControl_x), .aluOut(aluOut));
			mux_2x1_5b muxInstToReg(.in0(rt_x), .in1(rd_x), .sel(regDst_x), .out(writeAddr));
			mux_2x1_32b muxRegToAlu(.in0(muxIn1), .in1(imm32_x), .sel(aluSrc_x), .out(aluIn2));
			forwarding_unit fw(.regWrite_m(regWrite_m), .regWrite_b(regWrite_b), .writeAddr_m(writeAddr_m), .writeAddr_b(writeAddr_b), .rs_x(rs_x), .rt_x(rt_x), .forwardA(forwardA), .forwardB(forwardB));
            mux_3x1_32b muxALUin1(.in0(regOut1_x), .in1(writeData), .in2(aluOut_m), .sel(forwardA), .out(aluIn1));
            mux_3x1_32b muxALUin2(.in0(regOut2_x), .in1(writeData), .in2(aluOut_m), .sel(forwardB), .out(muxIn1));
            

		EXMEM EXMEMreg(.clk(clk), .regWrite_x(regWrite_x), .memToReg_x(memToReg_x), .memWrite_x(memWrite_x), .memRead_x(memRead_x), .writeAddr(writeAddr), .aluOut(aluOut), .regOut2_x(muxIn1), .regWrite_m(regWrite_m), .memToReg_m(memToReg_m), .memWrite_m(memWrite_m), .memRead_m(memRead_m), .writeAddr_m(writeAddr_m), .aluOut_m(aluOut_m), .regOut2_m(regOut2_m));
		

		//MEM
			data_mem DM(.clk(clk), .memWrite(memWrite_m), .memRead(memRead_m), .addr(aluOut_m), .memIn(regOut2_m), .memOut(memOut));


		MEMWB MEMWBreg(.clk(clk), .regWrite_m(regWrite_m), .memToReg_m(memToReg_m), .aluOut_m(aluOut_m), .memOut(memOut), .writeAddr_m(writeAddr_m), .regWrite_b(regWrite_b), .memToReg_b(memToReg_b), .aluOut_b(aluOut_b), .memOut_b(memOut_b), .writeAddr_b(writeAddr_b));


		//WB
			mux_2x1_32b muxMemToReg(.in0(aluOut_b), .in1(memOut_b), .sel(memToReg_b), .out(writeData));
endmodule




module IFID(
input clk, stall,
input [31:0] inst,

output reg [31:0] inst_d
);
always @(posedge clk)begin
        if(stall != 1'b1)begin
            inst_d <= inst;
        end
end
endmodule




module IDEX(
input clk, regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
input [31:0] regOut1, regOut2, imm32,
input [4:0] rt, rd, rs,
input [3:0] aluControl,

output reg regWrite_x, memToReg_x, memWrite_x, aluSrc_x, regDst_x, memRead_x,
output reg [31:0] regOut1_x, regOut2_x, imm32_x,
output reg [4:0] rt_x, rd_x, rs_x,
output reg [3:0] aluControl_x
);
always @(posedge clk)begin 
	regWrite_x <= regWrite;
	memToReg_x <= memToReg;
	memWrite_x <= memWrite;
	aluSrc_x <= aluSrc;
	regDst_x <= regDst;
	memRead_x <= memRead;
	regOut1_x <= regOut1;
	regOut2_x <= regOut2;
	imm32_x <= imm32;
	rt_x <= rt;
	rd_x <= rd;
	rs_x <= rs;
	aluControl_x <= aluControl;
end
endmodule




module EXMEM(
input clk, regWrite_x, memToReg_x, memWrite_x, memRead_x,
input [4:0] writeAddr,
input [31:0] aluOut, regOut2_x,

output reg regWrite_m, memToReg_m, memWrite_m, memRead_m,
output reg [4:0] writeAddr_m,
output reg [31:0] aluOut_m, regOut2_m
);
always @(posedge clk)begin
	regWrite_m <= regWrite_x;
	memToReg_m <= memToReg_x;
	memWrite_m <= memWrite_x;
	memRead_m <= memRead_x;
	writeAddr_m <= writeAddr;
	aluOut_m <= aluOut;
	regOut2_m <= regOut2_x;
end
endmodule




module MEMWB(
input clk, regWrite_m, memToReg_m,
input [31:0] aluOut_m, memOut,
input [4:0] writeAddr_m,

output reg regWrite_b, memToReg_b,
output reg [31:0] aluOut_b, memOut_b,
output reg [4:0] writeAddr_b
);
always @(posedge clk)begin
	regWrite_b <= regWrite_m;
	memToReg_b <= memToReg_m;
	aluOut_b <= aluOut_m;
	memOut_b <= memOut;
	writeAddr_b <= writeAddr_m;
end
endmodule







/* ================= Modules to implement for HW1 =====================*/
module program_counter(
    input clk, stall, 
    input [31:0] nextPc,
    output reg [31:0] pc
    );
    initial begin
        pc = 32'd96; // PC initialized to start from 100.
    end
    // ==================== Students fill here BEGIN ====================
    
    always @(posedge clk)begin 
        if(stall != 1'b1)begin
            pc <= nextPc; // sets pc equal to the next value(nextPc) at each positive clock edge
        end
    end

    // ==================== Students fill here END ======================
endmodule

module pc_adder(
    input [31:0] pc, offset,
    output reg [31:0] nextPc
    );
    // ==================== Students fill here BEGIN ==================== 
    always @(*)begin
        nextPc = pc + offset; //increments pc by 4(offset) and assigns it to nextPc
    end
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW3 =====================*/
module inst_mem(
    input [31:0] pc,
    output reg [31:0] inst
    );
    
    // This is an instruction memory that holds 64 instructions, 32b each.
    reg [31:0] memory [0:63];
    
    // Initializing instruction memory.
    initial begin       
        memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};
        memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};
        memory[27] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};
        memory[28] = {6'b100011, 5'd3, 5'd4, 16'hFFFC};
        
        //memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};
        //memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};
        //memory[27] = {6'b100011, 5'd0, 5'd3, 16'd8};
        //memory[28] = {6'b100011, 5'd0, 5'd4, 16'd16};
        //memory[29] = {6'b000000, 5'd1, 5'd2, 5'd5, 11'b00000100000};
        //memory[30] = {6'b100011, 5'd3, 5'd6, 16'hFFFC};
        //memory[31] = {6'b000000, 5'd4, 5'd3, 5'd7, 11'b00000100010};
        
        // memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};
        // memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};
        // memory[27] = {6'b100011, 5'd0, 5'd4, 16'd16};
        // memory[28] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};
        // memory[29] = {6'b100011, 5'd3, 5'd4, 16'hFFFC};
        
    end
    // ==================== Students fill here BEGIN ====================
    always @(*)begin 
        inst = memory[pc[31:2]];
    end
    // ==================== Students fill here END ======================
endmodule

module register_file(
    input [4:0] readAddr1, readAddr2, writeAddr,
    input [31:0] writeData, 
    input regWrite, clk,
    output reg [31:0] regOut1, regOut2
    );
    
    // Initializing registers. Do not touch here.
    reg [31:0] register [0:31]; // 32 registers, 32b each.
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) begin
            register[i] = 32'd0; // Initialize to zero
        end
    end
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        regOut1 = register[readAddr1];
        regOut2 = register[readAddr2];
    end
    always @(negedge clk) begin
	if(regWrite == 1) begin 
	 	register[writeAddr] <= writeData;
	end
    end
    // ==================== Students fill here END ======================
endmodule

module control_unit(
    input [5:0] op, func,
    input stall,
    output reg regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
    output reg [3:0] aluControl
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        // set control unit outputs based on opcode and func(only doing add, sub, lw, and sw as per HW instructions)
        case (op)
            6'b000000: begin 
                // R-type
                case (func)
                    6'b100000: begin 
                        //add
                        regWrite = 1'b1;
                        memToReg = 1'b0;
                        memWrite = 1'b0;
                        aluSrc = 1'b0;
                        regDst = 1'b1;
                        memRead = 1'b0;
                        aluControl = 4'b0010;
                    end
                    6'b100010: begin
                        //sub
                        regWrite = 1'b1;
                        memToReg = 1'b0;
                        memWrite = 1'b0;
                        aluSrc = 1'b0;
                        regDst = 1'b1;
                        memRead = 1'b0;
                        aluControl = 4'b0110;
                    end
                    default: begin 
                        //not add or sub but still R-type
                        regWrite = 1'b0;
                        memToReg = 1'b0;
                        memWrite = 1'b0;
                        aluSrc = 1'b0;
                        regDst = 1'b0;
                        memRead = 1'b0;
                        aluControl = 4'b0000;
                    end
                endcase
            end
            6'b100011: begin 
                //lw
                regWrite = 1'b1;
                memToReg = 1'b1;
                memWrite = 1'b0;
                aluSrc = 1'b1;
                regDst = 1'b0;
                memRead = 1'b1;
                aluControl = 4'b0010;
            end
            6'b101011: begin 
                //sw
                regWrite = 1'b0;
                memToReg = 1'bx;
                memWrite = 1'b1;
                aluSrc = 1'b1;
                regDst = 1'bx;
                memRead = 1'b0;
                aluControl = 4'b0010;
            end
            default: begin 
                //not an r type, lw, or sw
                regWrite = 1'b0;
                memToReg = 1'b0;
                memWrite = 1'b0;
                aluSrc = 1'b0;
                regDst = 1'b0;
                memRead = 1'b0;
                aluControl = 4'b0000;
            end
        endcase
        //set signals to 0 if stall is 1
        if(stall)begin
                regWrite = 1'b0;
                memToReg = 1'b0;
                memWrite = 1'b0;
                aluSrc = 1'b0;
                regDst = 1'b0;
                memRead = 1'b0;
                aluControl = 4'b0000;
        end
    end 
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW4 =====================*/
module imm_extend(
    input [15:0] imm,
    output reg [31:0] imm32
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin 
		imm32[15:0] = imm[15:0];
		case(imm[15:15])
			1'b0: begin
				imm32[31:16] = 16'h0000;
			end 
			1'b1: begin
				imm32[31:16] = 16'hffff;
			end
		endcase 
	end

    // ==================== Students fill here END ======================
endmodule

module mux_2x1_32b(
    input [31:0] in0, in1,
    input sel,
    output reg [31:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin 
		case(sel)
			1'b0: begin
				out = in0;
			end
			1'b1: begin
				out = in1;
			end
		endcase
	end
    // ==================== Students fill here END ======================
endmodule

module alu(
    input [31:0] aluIn1, aluIn2,
    input [3:0] aluControl,
    output reg [31:0] aluOut
    );

    // ==================== Students fill here BEGIN ====================
    always @(*) begin 
		case(aluControl) 
			4'b0000: begin 
				//bitwise AND
				aluOut = aluIn1 & aluIn2;
			end
			4'b0001: begin 
				//bitwise OR
				aluOut = aluIn1 | aluIn2;
			end
			4'b0010: begin 
				//add
				aluOut = aluIn1 + aluIn2;
			end
			4'b0110: begin 
				//subtract 
				aluOut = aluIn1 - aluIn2;
			end
			4'b0111: begin 
				// set if less than
				if (aluIn1 < aluIn2) begin 
					aluOut = 32'h00000001;
				end else begin 
					aluOut = 32'h00000000;
				end
			end
			4'b1100: begin 
				// bitwise NOR
				aluOut = ~(aluIn1 | aluIn2);
			end
			4'b1000: begin 
				// shift left logical
				aluOut = aluIn1 << aluIn2;
			end
			4'b1001: begin 
				// shift right logical 
				aluOut = aluIn1 >> aluIn2;
			end
		endcase
	end

    // ==================== Students fill here END ======================
endmodule

module data_mem(
    input clk, memWrite, memRead,
    input [31:0] addr, memIn,
    output reg [31:0] memOut
    );
    
    reg [31:0] memory [0:63]; // 64x32 memory
    
    // Initialize data memory. Do not touch this part.
    initial begin
        memory[0] = 32'd16817;
        memory[1] = 32'd16801;
        memory[2] = 32'd16;
        memory[3] = 32'hDEAD_BEEF;
        memory[4] = 32'h4242_4242;
    end
    
 
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
		case(memRead)
			1'b0: begin 
				memOut = 32'dx;
			end
			1'b1: begin 
				memOut = memory[addr[31:2]];
			end
		endcase
	end
     always @(negedge clk) begin
		case(memWrite)
			1'b0: begin 
				//do nothing
			end
			1'b1: begin 
				memory[addr[31:2]] <= memIn;
			end
		endcase
	end
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW5 =====================*/
module mux_2x1_5b(
    input [4:0] in0, in1,
    input sel,
    output reg [4:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin 
		case(sel)
			1'b0: begin
				out = in0;
			end
			1'b1: begin
				out = in1;
			end
		endcase
	end
    // ==================== Students fill here END ======================
endmodule

module mux_3x1_32b(
    input [31:0] in0, in1, in2,
    input [1:0] sel,
    output reg [31:0] out
    );
    always @(*) begin 
        case(sel)
            2'b00: begin
                out = in0;
            end
            2'b01: begin
                out = in1;
            end
            2'b10: begin
                out = in2;
            end
        endcase
    end
endmodule


module forwarding_unit(
    input regWrite_m, regWrite_b,
    input [4:0] writeAddr_m, writeAddr_b, rs_x, rt_x,
    output reg [1:0] forwardA, forwardB
);
    always @(*) begin
        // Forwarding controls to 0(no forwardng)
        forwardA = 2'b00;
        forwardB = 2'b00;
// EX/MEM Forwarding
        if (regWrite_m && (writeAddr_m != 5'b00000) && (writeAddr_m == rs_x))begin
            forwardA = 2'b10;
        end 


        if ((regWrite_m != 0) && (writeAddr_m != 5'b00000) && (writeAddr_m == rt_x) )begin
            forwardB = 2'b10;
        end 
// MEM/WB Forwarding
        if (regWrite_b 
        && (writeAddr_b != 5'b00000) 
        && !(regWrite_m && (writeAddr_m != 5'b00000)
        && (writeAddr_m == rs_x))
        && (writeAddr_b == rs_x))
        begin
            forwardA = 2'b01;
        end 


        if (regWrite_b 
        && (writeAddr_b != 5'b00000) 
        && !(regWrite_m && (writeAddr_m != 5'b00000)
        && (writeAddr_m == rt_x))
        && (writeAddr_b == rt_x))
        begin
            forwardB = 2'b01;
        end
    end
endmodule


module hazard_unit(
    input memRead_x,
    input [4:0] rt_x, rt, rs,
    output reg stall
);
    initial begin
        stall = 1'b0;
    end
    
    always @(*)begin 
        if(memRead_x
        &&((rt_x == rs)
        || (rt_x == rt)))
        begin  
            stall = 1'b1;
        end else begin 
            stall = 1'b0;
        end
    end
endmodule








