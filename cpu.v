`default_nettype none
module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
    wire branchBlt,branchBeq,branchJal,branchJalr,regWrite,memToReg,/*memWrite*/ALUSrc;
    wire [2:0] ALUControl;
    wire [2:0] ImmControl;

    wire [31:0] res; //from adder from data memory 
    wire [31:0] rs1;//, rs2;

    wire [31:0] immOp;

    //wire [31:0] ALUOut;
    wire zero;
    wire aSmaller;

    wire [31:0] srcB;

    wire [31:0] constFour = 4;

    wire [31:0] PCn;

    wire [31:0] PCPlus4;

    wire [31:0] sum2Out;

    wire [31:0] branchTarget;

    wire [31:0] mux5Out;

    wire branchJalx;
    wire branchOutcome;

    controlUnit cUnit (instruction,branchBlt,branchBeq,branchJal,branchJalr,regWrite,memToReg,WE,ALUControl,ALUSrc, ImmControl);
    soubor3Port reg3(instruction [19:15],instruction [24:20], instruction [11:7], 
                   clk, regWrite, 
                   res, 
                   rs1,data_to_mem);
    immDecode iDecode (instruction[31:7], ImmControl, immOp);
    alu32Bit alu(rs1, srcB, 
                ALUControl, 
                address_to_mem, 
                zero,
                aSmaller
               );
    branchJalxCirc branchJalxC (branchJal, branchJalr, branchJalx);
    branchOutcomeCirc branchOutcomeC (branchBlt, zero, aSmaller, branchBeq, branchJalx, branchOutcome);           
    multiplexor mux1 (PCPlus4,branchTarget,branchOutcome,PCn);          
    pcRegistr pcReg (clk,reset, PCn,  PC);
    
    multiplexor mux2 (data_to_mem,immOp,ALUSrc, srcB);
    multiplexor mux3 (sum2Out, address_to_mem, branchJalr, branchTarget);
    multiplexor mux4 (mux5Out,data_from_mem,memToReg, res);
    multiplexor mux5 (address_to_mem,PCPlus4, branchJalx, mux5Out);
    adder sum1 (constFour,PC,PCPlus4);
    adder sum2 (immOp, PC, sum2Out);

    

endmodule

//... add new modules here ...
module alu32Bit(  input [31:0] SrcA , SrcB, 
                  input [2:0] ALUControl, 
                  output reg [31:0] ALUResult, 
                  output reg Zero,
                  output reg aSmaller
               );

    always@(*) begin 
        if (ALUControl == 0) ALUResult = SrcA + SrcB;
        if (ALUControl== 1) ALUResult = SrcA - SrcB;
        if (ALUControl== 2) ALUResult = SrcA & SrcB;
        if (ALUControl== 3) ALUResult = $unsigned(SrcA) >> SrcB;
        if (ALUControl== 4) begin
            ALUResult[31:24] = SrcA[31:24] + SrcB[31:24];
            ALUResult[23:16] = SrcA[23:16] + SrcB[23:16];
            ALUResult[15:8] = SrcA[15:8] + SrcB[15:8];
            ALUResult[7:0] = SrcA[7:0] + SrcB[7:0];
        end
        if (ALUControl== 5) begin
            ALUResult[31:24] = (SrcA[31:24] + SrcB[31:24])/2;
            ALUResult[23:16] = (SrcA[23:16] + SrcB[23:16])/2;
            ALUResult[15:8] = (SrcA[15:8] + SrcB[15:8])/2;
            ALUResult[7:0] = (SrcA[7:0] + SrcB[7:0])/2;
        end
        if (ALUResult == 0) Zero =1;
        else Zero = 0;
        if ($signed(SrcA) < $signed(SrcB)) aSmaller = 1;
        else aSmaller = 0;
    end
endmodule

module soubor3Port(input [4:0] A1 , A2, A3, 
                   input clk, WE3,
                   input [31:0] WD3, 
                   output reg [31:0] rd1,rd2
                  );

    reg [31:0] rf[31:1];
    
    always@(*) begin 
        if (A1 != 0 ) rd1 = rf[A1] ;
        else rd1 = 0;

        if (A2 != 0 ) rd2 = rf[A2];
        else rd2 = 0;
    end

    always@(posedge clk) begin 
        if (WE3 == 1 && A3 != 0) rf[A3] = WD3;
    end
endmodule

module pcRegistr (input clk, reset,
                  input [31:0] PCn, 
                  output reg [31:0] PC
                 );
    always@(posedge clk or posedge reset) begin 
        if (reset) PC <= 0;
        else PC <= PCn;
    end

endmodule

module controlUnit(input [31:0] instruction,
                   output reg branchBlt,
                   output reg branchBeq,
                   output reg branchJal,
                   output reg branchJalr,
                   output reg regWrite,
                   output reg memToReg,
                   output reg memWrite,
                   output reg [2:0] ALUControl,
                   output reg ALUSrc,
                   output reg [2:0] ImmControl //ikd jestli staci 3 bity
                  );
    always@(*) begin 
        if (instruction[6:0] == 7'b0110011) begin //add,sub,and,srl
            branchBeq <= 0;
            branchBlt <= 0;
            branchJal <= 0;
            branchJalr <= 0;
            regWrite <= 1;
            memToReg <= 0;
            memWrite <= 0;
            ALUSrc <= 0;
            ImmControl <= 0; //X;
            if (instruction[31:25] == 7'b0000000 && instruction[14:12] == 3'b000) ALUControl <= 0;
            if (instruction[31:25] == 7'b0000000 && instruction[14:12] == 3'b111) ALUControl <= 2;
            if (instruction[31:25] == 7'b0100000 && instruction[14:12] == 3'b000) ALUControl <= 1;
            if (instruction[31:25] == 7'b0000000 && instruction[14:12] == 3'b101) ALUControl <= 3;

        end
        if (instruction[6:0] == 7'b0010011) begin //addi
            branchBeq <= 0;
            branchBlt <= 0;
            branchJal <= 0;
            branchJalr <= 0;
            regWrite <= 1;
            memToReg <= 0;
            memWrite <= 0;
            ALUControl <= 0;
            ALUSrc <= 1;
            ImmControl <= 1;
        end
        if (instruction[6:0] == 7'b1100011) begin //beq,blt 
            branchJal <= 0;
            branchJalr <= 0;
            regWrite <= 0;
            memToReg <= 0;//x;
            memWrite <= 0;
            ALUControl <= 1;
            ALUSrc <= 0;
            ImmControl <=3;
            if (instruction[14:12] == 3'b000) begin
                branchBeq <= 1;
                branchBlt <= 0;
            end
            if (instruction[14:12] == 3'b100) begin
                branchBeq <= 0;
                branchBlt <= 1;
            end

        end
        if (instruction[6:0] == 7'b0000011) begin //lw 
             branchBeq <= 0;
            branchBlt <= 0;
            branchJal <= 0;
            branchJalr <= 0;
            regWrite <= 1;
            memToReg <= 1;
            memWrite <= 0;
            ALUControl <= 0;
            ALUSrc <= 1;
            ImmControl <= 1;
        end
        if (instruction[6:0] == 7'b0100011) begin //sw
            branchBeq <= 0;
            branchBlt <= 0;
            branchJal <= 0;
            branchJalr <= 0;
            regWrite <= 0;
            memToReg <= 1;
            memWrite <= 1;
            ALUControl <= 0;
            ALUSrc <= 1;
            ImmControl <= 2; 
        end
        if (instruction[6:0] == 7'b1101111) begin //jal 
            branchBeq <= 0;
            branchBlt <= 0;
            branchJal <= 1;
            branchJalr <= 0;
            regWrite <= 1;
            memToReg <= 0;
            memWrite <= 0;
            ALUControl <= 0;//x;
            ALUSrc <= 0;//x;
            ImmControl <= 5; 
        end
        if (instruction[6:0] == 7'b1100111) begin //jalr
            branchBeq <= 0;
            branchBlt <= 0;
            branchJal <= 0;
            branchJalr <= 1;
            regWrite <= 1;
            memToReg <= 0;
            memWrite <= 0;
            ALUControl <= 0;
            ALUSrc <= 1;
            ImmControl <= 1; 
        end
        if (instruction[6:0] == 7'b0001011) begin //add_v,avg_v
            branchBeq <= 0;
            branchBlt <= 0;
            branchJal <= 0;
            branchJalr <= 0;
            regWrite <= 1;
            memToReg <= 0;
            memWrite <= 0;
            ALUSrc <= 0;
            ImmControl <= 0;//x;
            if (instruction[31:25] == 7'b0000000 && instruction[14:12] == 3'b000) ALUControl <= 4;
            if (instruction[31:25] == 7'b0000000 && instruction[14:12] == 3'b001) ALUControl <= 5;
        end
    end
endmodule

module immDecode (input [24:0] immInstruction, 
                  input [2:0] immControl,
                  output reg[31:0] immOp);
    always@(*) begin 
        if (immControl == 0) begin
            immOp = 0;
        end
        if (immControl == 1) begin
            immOp = 0;
            immOp[11:0] = immInstruction[24:13];
        end
        if (immControl == 2) begin
            immOp = 0;
            immOp[11:5] = immInstruction[24:18];
            immOp[4:0] = immInstruction[4:0];
        end
        if (immControl == 3) begin
            immOp = 0;
            immOp[12] = immInstruction[24];
            immOp[10:5] = immInstruction[23:18];
            immOp[4:1] = immInstruction[4:1];
            immOp[11] = immInstruction[0];
            
        end
        if (immControl == 4) begin
            immOp = 0;
            immOp[19:0] = immInstruction[24:5];
        end
        if (immControl == 5) begin
            immOp = 0;
            immOp[20] = immInstruction[24];
            immOp[10:1] = immInstruction[23:14];
            immOp[11] = immInstruction[13];
            immOp[19:12] = immInstruction[12:5];
        end
    end
endmodule

module multiplexor (input [31:0] in0, in1, 
             input select,
             output reg [31:0] out
            );
    always@(*) begin
        if (select == 1) out = in1;
        else out = in0;
    end
endmodule

module adder (input [31:0] in0, in1,
              output [31:0] out);

    assign out = in0 + in1;
    
endmodule

module branchJalxCirc (input branchJal, branchJalr,
              output branchJalx);

    assign branchJalx = branchJal | branchJalr;
    
endmodule

module branchOutcomeCirc (input branchBlt, ALUzero, ALUaSmaller, branchBeq, branchJalx,
                          output branchOutcome);

    assign branchOutcome = (branchBlt & ALUaSmaller) | ((branchBeq & ALUzero)| branchJalx);
    
endmodule
`default_nettype wire