//Subject:     Architecture project 2 - ALU Controller
//--------------------------------------------------------------------------------
//Version:     1
//--------------------------------------------------------------------------------
//Writer:      
//----------------------------------------------
//Date:        
//----------------------------------------------
//Description: 
//--------------------------------------------------------------------------------

module ALU_Ctrl(
          funct_i,
          ALUOp_i,
          ALUCtrl_o
          );
          
//I/O ports 
input      [6-1:0] funct_i;
input      [3-1:0] ALUOp_i;

output     [4-1:0] ALUCtrl_o;    
     
//Internal Signals
reg        [4-1:0] ALUCtrl_o;

//Parameter


//Select exact operation, please finish the following code
always@(funct_i or ALUOp_i) begin
    case(ALUOp_i)
        3'b000: begin
			ALUCtrl_o = 4'b0010 ; // lw
        end
		3'b001: begin
			ALUCtrl_o = 4'b0010 ;
		end
		3'b010: begin
			case (funct_i)
				6'b100000: ALUCtrl_o = 4'b0010;//add
				6'b100010: ALUCtrl_o = 4'b0110;//substract
				6'b100100: ALUCtrl_o = 4'b0000; // AND
				6'b100101: ALUCtrl_o = 4'b0001;// or
				6'b101010: ALUCtrl_o = 4'b0111;// set on less than	
			endcase
		end
        3'b011 : begin // addi
			ALUCtrl_o = 4'b0010 ; //addi
		end
		3'b100 : begin  // slti
			ALUCtrl_o = 4'b0111; 
		end
    endcase
end
endmodule
