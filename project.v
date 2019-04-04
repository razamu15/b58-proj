module project
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        LEDR,
        SW,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		HEX0,                      // TODO: Take this out, this is for debugging
	CLOCK_27,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	I2C_SCLK
	);

	input			CLOCK_50;				//	50 MHz
	input   [17:0]   SW;
	output   [17:0]   LEDR;

	output [6:0] HEX0;
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	
	
	

	input				CLOCK_27;
	input		[3:0]	KEY;
	input				AUD_ADCDAT;

// Bidirectionals
	inout				AUD_BCLK;
	inout				AUD_ADCLRCK;
	inout				AUD_DACLRCK;

	inout				I2C_SDAT;

// Outputs
	output				AUD_XCK;
	output				AUD_DACDAT;

	output				I2C_SCLK;
	
	
	
	
	
	
	
	wire resetn;
	assign resetn = SW[17];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	//assign writeEn = SW[16];
	assign x = 8'b01010101;
	assign y = 7'b0101010;

	wire [7:0] x_plot;
	wire [6:0] y_plot;
	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x_plot),
			.y(y_plot),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
    // Instansiate datapath
	// datapath d0(...);

    // Instansiate FSM control
    // control c0(...);
	 wire tile1, tile2;
	 reg [3:0] music;
	 
	 assign tile1 = KEY[2];
	 assign tile2 = KEY[1];
	 
//	 always @(*)
//	begin
//		if (y_plot > 7'd116 && x_plot < 7'd3 && tile1 == 1'b0)
//		begin
//			music = 4'b1000;
//		end
//		else begin
//			 if (y_plot > 7'd116 && x_plot > 7'd3 && tile2 == 1'b0) begin
//				music = 4'b0001;
//			end
//			else begin
//				music = 4'b0000;
//			end
//		end
//	end

always @(*)
	begin
		if (y_plot < 7'd116 && x_plot > 7'd3)
		begin
			music = 4'b0000;
		end
		else begin
			 if (y_plot < 7'd116 && x_plot > 7'd3) begin
				music = 4'b0000;
			end
			else begin
				if (tile1 == 1'b0 || tile2 == 1'b0)
				begin
				music = 4'b0100;
				end
			end
		end
	end

	 
	wire clock;
	wire [19:0] clock_iteration;
	wire [5:0] pos_iteration;
	// Wires connecting starting position and control path
	wire [7:0] x_temp;
	wire [6:0] y_temp;
	
	music( .CLOCK_50(CLOCK_50), .CLOCK_27(CLOCK_27), .KEY(KEY), .AUD_ADCDAT(AUD_ADCDAT),	.AUD_BCLK(AUD_BCLK), .AUD_ADCLRCK(AUD_ADCLRCK), .AUD_DACLRCK(AUD_DACLRCK), .I2C_SDAT(I2C_SDAT), .AUD_XCK(AUD_XCK),	.AUD_DACDAT(AUD_DACDAT),	.I2C_SCLK(I2C_SCLK), .SW(music));
	modified_clock mc (.hit_cycle(clock), .cur_iter(clock_iteration), .next_iter(clock_iteration), .clock(CLOCK_50));
	// Starting position is for the shift down of all columns (they shift down synchronously)
	starting_pos sp (.x_start(x), .y_start(y), .x(x_temp), .y(y_temp), .x_new(x_temp), .y_new(y_temp), .cur_iter(pos_iteration), .next_iter(pos_iteration), .clock(clock));
	control_path cp (.clock(CLOCK_50), .x(x_temp), .x_out(x_plot), .y(y_temp), .y_out(y_plot), .colour(colour), .plot(writeEn), .pos_counter(pos_iteration), .check_plot(LEDR[17]));

endmodule



module control_path(
    input clock,
    input resetn,
//	 input go,
    input [7:0] x,
    input [6:0] y,
	 input [5:0] pos_counter,
    output reg [2:0] colour,
	 output reg  plot,
    output reg [7:0] x_out,
    output reg [6:0] y_out,
	 output reg check_plot
    );
	
	 reg go;
    reg [4:0] current_state, next_state; 
    
    localparam  S_DRAW_C1 = 4'd0,
					S_DRAW_C2 = 4'd1;
					//S_DRAW_C3 = 4'd2;
	
	// this always bloack defines colour absed on the when we are going to change coordinates for the columns shifting down
	always @(*)
	begin
		if (pos_counter > 6'b101111 || pos_counter < 6'b000100)
		begin
			colour = 3'b000;
		end
		else begin
		   colour = 3'b111;
		end
	end

    // Next state logic aka our state table
    always@(*) //Verify this FSM makes sense transition wise
    begin: state_table 
            case (current_state)
			// Move on the erase state if all shifts have
			// been applied (x = x + 1, y = y + 1)
			S_DRAW_C1: next_state =  (go) ? S_DRAW_C2 : S_DRAW_C1;
			S_DRAW_C2: next_state = (go) ? S_DRAW_C1 : S_DRAW_C2;
         default:   next_state = S_DRAW_C1;
        endcase
    end // state_table
   

    // Output logic aka all of our datapath control signals
    always @(posedge clock)
    begin: enable_signals
        // By default make all our signals 0
        plot = 1'b1;
        case (current_state)
 			S_DRAW_C1:
			begin // Taking the given values and plotting
				x_out = x + pos_counter[0];
				y_out = y + pos_counter[1];
				plot = 1'b1;
				check_plot = plot;
 			end
			S_DRAW_C2:
			begin // Taking the given values and plotting
				x_out = x + pos_counter[0] + 3'b100;
				y_out = y + pos_counter[1] + 3'b110;
				plot = 1'b1;
				check_plot = plot;
 			end
			default: 
			begin
			plot = 1'b0; // TODO: CHANGE TO 0
			colour = 3'b011;
			end 
        endcase
		  
    end // enable_signals
	 
	 
	 always @(*)
	begin
		if (pos_counter[1:0] == 2'b11)
		begin
			go <= ~go;
		end
	end
	 
	 
always @(posedge clock)
begin: state_FFs
current_state <= next_state;
end
	 
 endmodule


module modified_clock (hit_cycle, cur_iter, next_iter, clock);
input clock;
input [19:0] cur_iter; // the current value in the iteration
output reg [19:0] next_iter; // next iteration's value
output reg hit_cycle; // reg telling us whether this clock cycle hit the cycle
localparam goal_speed = 20'b00011011011100110101;
always @(posedge clock)
begin
    if(cur_iter == goal_speed)  begin
    hit_cycle <= 1'b1; // then we hit our cycle, output 1
    next_iter <= 20'b00000000000000000000; // start counting again in the next iteration
    end
    else begin
    hit_cycle <= 1'b0; // we didn't hitthe cycle, increment and try again
    next_iter <= cur_iter + 1'b1;
    end
end
endmodule


module hex_display(IN, OUT);
    input [3:0] IN;
	 output reg [7:0] OUT;
	 
	 always @(*)
	 begin
		case(IN[3:0])
			4'b0000: OUT = 7'b1000000;
			4'b0001: OUT = 7'b1111001;
			4'b0010: OUT = 7'b0100100;
			4'b0011: OUT = 7'b0110000;
			4'b0100: OUT = 7'b0011001;
			4'b0101: OUT = 7'b0010010;
			4'b0110: OUT = 7'b0000010;
			4'b0111: OUT = 7'b1111000;
			4'b1000: OUT = 7'b0000000;
			4'b1001: OUT = 7'b0011000;
			4'b1010: OUT = 7'b0001000;
			4'b1011: OUT = 7'b0000011;
			4'b1100: OUT = 7'b1000110;
			4'b1101: OUT = 7'b0100001;
			4'b1110: OUT = 7'b0000110;
			4'b1111: OUT = 7'b0001110;
			
			default: OUT = 7'b0111111;
		endcase

	end
endmodule

module starting_pos (x_start, y_start, x, y, x_new, y_new, cur_iter, next_iter, clock);
input clock;
input [7:0] x_start, x;
input [6:0] y_start, y;
output reg [7:0] x_new;
output reg [6:0] y_new;
input [5:0] cur_iter; // the current value in the iteration
output reg [5:0] next_iter; // next iteration's value
// box moving down speed
localparam goal_speed = 6'b111111;
always @(posedge clock)
begin
    if(cur_iter == goal_speed)  begin
    x_new <= x;
    y_new <= y  + 3'b001; // then we hit our cycle, output 1
    next_iter <= 6'b000000; // start counting again in the next iteration
    end
    else begin
    x_new <= x; // we didn't hitthe cycle, increment and try again
    y_new <= y;
    next_iter <= cur_iter + 1'b1;
    end
end
endmodule









