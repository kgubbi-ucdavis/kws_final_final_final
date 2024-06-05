// SPDX-License-Identifier: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_project_wrapper
 *
 * This wrapper enumerates all of the pins available to the
 * user for the user project.
 *
 * An example user project is provided in this wrapper.  The
 * example should be removed and replaced with the actual
 * user project.
 *
 *-------------------------------------------------------------
 */

module user_project_wrapper #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vdda1,    // User area 1 3.3V supply
    inout vdda2,    // User area 2 3.3V supply
    inout vssa1,    // User area 1 analog ground
    inout vssa2,    // User area 2 analog ground
    inout vccd1,    // User area 1 1.8V supply
    inout vccd2,    // User area 2 1.8v supply
    inout vssd1,    // User area 1 digital ground
    inout vssd2,    // User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

    // Internal wires
    wire clk;
    wire reset;
    wire start;
    wire [7:0] serial_weight_data;
    wire serial_weight_valid;
    wire [7:0] serial_line_data;
    wire serial_line_valid;
    wire [7:0] serial_result;
    wire serial_result_valid;
    wire done;

    // Assignments for connecting the IOs to the internal signals
    assign clk = wb_clk_i;
    assign reset = wb_rst_i;
    assign start = la_data_in[0];
    assign serial_weight_data = la_data_in[39:32];
    assign serial_weight_valid = la_data_in[40];
    assign serial_line_data = la_data_in[48:41];
    assign serial_line_valid = la_data_in[49];
    assign la_data_out[7:0] = serial_result;
    assign la_data_out[8] = serial_result_valid;
    assign la_data_out[9] = done;

    // Instantiate the top-level project
    CNN_Accelerator_Top mprj (
`ifdef USE_POWER_PINS
        .vccd1(vccd1),  // User area 1 1.8V power
        .vssd1(vssd1),  // User area 1 digital ground
`endif
        .clk(clk),
        .reset(reset),
        .start(start),
        .serial_weight_data(serial_weight_data),
        .serial_weight_valid(serial_weight_valid),
        .serial_line_data(serial_line_data),
        .serial_line_valid(serial_line_valid),
        .serial_result(serial_result),
        .serial_result_valid(serial_result_valid),
        .done(done)
    );

endmodule   // user_project_wrapper


module CNN_Accelerator_Top(
    `ifdef USE_POWER_PINS
        inout vccd1,    // User area 1 1.8V supply
        inout vssd1,    // User area 1 digital ground
    `endif
    input wire clk,
    input wire reset,
    input wire start,
    input wire [7:0] serial_weight_data,  // Serial weight data from off-chip memory
    input wire serial_weight_valid,       // Valid signal for serial weight data
    input wire [7:0] serial_line_data,    // Serial line data from off-chip memory
    input wire serial_line_valid,         // Valid signal for serial line data
    output reg [7:0] serial_result,       // Serial output for result data
    output reg serial_result_valid,       // Valid signal for serial result data
    output wire done
);
    
    // Registers to store the results
    reg [7:0] result [15:0];
    reg [3:0] result_index;

    // Control FSM signals
    wire weight_write_enable;
    wire line_write_enable;
    wire [7:0] weight_data;
    wire [3:0] weight_write_addr;
    wire [7:0] line_data;
    wire [3:0] line_write_addr;
    wire [3:0] line_read_addr;

    // Intermediate signals
    wire [127:0] weight_out;
    wire [127:0] line_out;
    wire [255:0] products;
    wire [127:0] sum;

    // Instantiate the Control FSM
    ControlFSM control_fsm (
        .clk(clk),
        .reset(reset),
        .start(start),
        .weight_write_enable(weight_write_enable),
        .line_write_enable(line_write_enable),
        .weight_data(weight_data),
        .weight_write_addr(weight_write_addr),
        .line_data(line_data),
        .line_write_addr(line_write_addr),
        .line_read_addr(line_read_addr),
        .done(done),
        .serial_weight_data(serial_weight_data),
        .serial_weight_valid(serial_weight_valid),
        .serial_line_data(serial_line_data),
        .serial_line_valid(serial_line_valid)
    );

    // Instantiate the Weight Buffer
    WeightBuffer weight_buffer (
        .clk(clk),
        .reset(reset),
        .weight_data(weight_data),
        .write_enable(weight_write_enable),
        .write_addr(weight_write_addr),
        .weight_out(weight_out)
    );

    // Instantiate the Line Buffer
    LineBuffer line_buffer (
        .clk(clk),
        .reset(reset),
        .data_in(line_data),
        .write_enable(line_write_enable),
        .write_addr(line_write_addr),
        .read_addr(line_read_addr),
        .data_out(line_out)
    );

    // Instantiate the MAC Array
    MACArray mac_array (
        .line_data(line_out),
        .weight_data(weight_out),
        .products(products)
    );

    // Instantiate the Configurable Adder Tree
    ConfigurableAdderTree adder_tree (
        .products(products),
        .final_sum(sum)
    );

    // Combined always block for state transitions and signal assignments
    localparam IDLE = 2'b00,
               SERIALIZE = 2'b01;

    reg [1:0] state, next_state;
    integer i;  // Declare the loop variable here

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            next_state <= IDLE;
            result_index <= 0;
            serial_result_valid <= 0;
            serial_result <= 8'b0;
            for (i = 0; i < 16; i = i + 1) begin
                result[i] <= 0;
            end
        end else begin
            state <= next_state;
            case (state)
                IDLE: begin
                    if (done) begin
                        next_state <= SERIALIZE;
                        for (i = 0; i < 16; i = i + 1) begin
                            result[i] <= sum[i*8 +: 8];
                        end
                        result_index <= 0;
                    end else begin
                        serial_result_valid <= 0;
                    end
                end
                SERIALIZE: begin
                    if (result_index < 16) begin
                        serial_result <= result[result_index];
                        serial_result_valid <= 1;
                        result_index <= result_index + 1;
                    end else begin
                        next_state <= IDLE;
                        serial_result_valid <= 0;
                    end
                end
                default: next_state <= IDLE;
            endcase
        end
    end
endmodule

module ControlFSM(
    input wire clk,
    input wire reset,
    input wire start,
    output reg weight_write_enable,
    output reg line_write_enable,
    output reg [7:0] weight_data,
    output reg [3:0] weight_write_addr,
    output reg [7:0] line_data,
    output reg [3:0] line_write_addr,
    output reg [3:0] line_read_addr,
    output reg done,
    input wire [7:0] serial_weight_data,  // Serial weight data from top module
    input wire serial_weight_valid,       // Valid signal for serial weight data
    input wire [7:0] serial_line_data,    // Serial line data from top module
    input wire serial_line_valid          // Valid signal for serial line data
);
    reg [3:0] state, next_state;
    reg [3:0] read_addr_counter; // Read address counter

    localparam IDLE = 4'b0000,
               LOAD_WEIGHTS = 4'b0001,
               LOAD_LINE = 4'b0010,
               COMPUTE = 4'b0011,
               RELU = 4'b0101,
               POOL = 4'b0110,
               DONE = 4'b0111;

    // Sequential logic for state transitions and assignments
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            read_addr_counter <= 0; // Initialize read address counter
            weight_write_addr <= 0; // Initialize write address
            line_write_addr <= 0;   // Initialize write address
            weight_write_enable <= 0;
            line_write_enable <= 0;
            weight_data <= 0;
            line_data <= 0;
            line_read_addr <= 0;
            done <= 0;
        end else begin
            state <= next_state;
            line_read_addr <= read_addr_counter; // Dynamic read address

            case (state)
                IDLE: begin
                    if (start)
                        next_state <= LOAD_WEIGHTS;
                end
                LOAD_WEIGHTS: begin
                    if (serial_weight_valid) begin
                        weight_write_enable <= 1;
                        weight_data <= serial_weight_data;
                        weight_write_addr <= weight_write_addr + 1;
                        if (weight_write_addr == 4'b1111)
                            next_state <= LOAD_LINE;
                    end else begin
                        weight_write_enable <= 0;
                    end
                end
                LOAD_LINE: begin
                    if (serial_line_valid) begin
                        line_write_enable <= 1;
                        line_data <= serial_line_data;
                        line_write_addr <= line_write_addr + 1;
                        if (line_write_addr == 4'b1111)
                            next_state <= COMPUTE;
                    end else begin
                        line_write_enable <= 0;
                    end
                end
                COMPUTE: begin
                    next_state <= RELU;
                    read_addr_counter <= read_addr_counter + 1;
                end
                RELU: begin
                    next_state <= POOL;
                end
                POOL: begin
                    next_state <= DONE;
                end
                DONE: begin
                    done <= 1;
                    next_state <= IDLE;
                end
                default: next_state <= IDLE; // Ensure default case is handled
            endcase
        end
    end
endmodule

module ReLU(
    input wire [127:0] data_in,
    output wire [127:0] data_out
);
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : relu_loop
            assign data_out[i*8 +: 8] = (data_in[i*8 + 7] == 1'b1) ? 8'b0 : data_in[i*8 +: 8]; // Check if the MSB (sign bit) is 1
        end
    endgenerate
endmodule

module Pooling(
    input wire [7:0] data_in0,
    input wire [7:0] data_in1,
    input wire [7:0] data_in2,
    input wire [7:0] data_in3,
    output wire [7:0] data_out
);
    assign data_out = (data_in0 + data_in1 + data_in2 + data_in3) >> 2; // Average pooling
endmodule

module AdderTree(
    input wire [7:0] in0,
    input wire [7:0] in1,
    input wire [7:0] in2,
    input wire [7:0] in3,
    input wire [7:0] in4,
    input wire [7:0] in5,
    input wire [7:0] in6,
    input wire [7:0] in7,
    output wire [7:0] sum
);
    wire [7:0] sum1, sum2, sum3, sum4;
    assign sum1 = in0 + in1;
    assign sum2 = in2 + in3;
    assign sum3 = in4 + in5;
    assign sum4 = in6 + in7;
    assign sum = sum1 + sum2 + sum3 + sum4;
endmodule

module ConfigurableAdderTree(
    input wire [255:0] products, // Flattened array for products (16 x 16-bit)
    output wire [127:0] final_sum // Flattened array for final sums (16 x 8-bit)
);

    // Unflattened products for easier readability
    wire [15:0] products_array [15:0];
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : products_loop
            assign products_array[i] = products[i*16 +: 16]; // Fixed range
        end
    endgenerate

    // Pointwise summation
    AdderTree adder_tree_0 (
        .in0(products_array[0][7:0]),
        .in1(products_array[1][7:0]),
        .in2(products_array[2][7:0]),
        .in3(products_array[3][7:0]),
        .in4(products_array[4][7:0]),
        .in5(products_array[5][7:0]),
        .in6(products_array[6][7:0]),
        .in7(products_array[7][7:0]),
        .sum(final_sum[7:0])
    );

    AdderTree adder_tree_1 (
        .in0(products_array[8][7:0]),
        .in1(products_array[9][7:0]),
        .in2(products_array[10][7:0]),
        .in3(products_array[11][7:0]),
        .in4(products_array[12][7:0]),
        .in5(products_array[13][7:0]),
        .in6(products_array[14][7:0]),
        .in7(products_array[15][7:0]),
        .sum(final_sum[15:8])
    );

    AdderTree adder_tree_2 (
        .in0(products_array[0][15:8]),
        .in1(products_array[1][15:8]),
        .in2(products_array[2][15:8]),
        .in3(products_array[3][15:8]),
        .in4(products_array[4][15:8]),
        .in5(products_array[5][15:8]),
        .in6(products_array[6][15:8]),
        .in7(products_array[7][15:8]),
        .sum(final_sum[23:16])
    );

    AdderTree adder_tree_3 (
        .in0(products_array[8][15:8]),
        .in1(products_array[9][15:8]),
        .in2(products_array[10][15:8]),
        .in3(products_array[11][15:8]),
        .in4(products_array[12][15:8]),
        .in5(products_array[13][15:8]),
        .in6(products_array[14][15:8]),
        .in7(products_array[15][15:8]),
        .sum(final_sum[31:24])
    );

    AdderTree adder_tree_4 (
        .in0(products_array[0][7:0]),
        .in1(products_array[1][7:0]),
        .in2(products_array[2][7:0]),
        .in3(products_array[3][7:0]),
        .in4(products_array[4][7:0]),
        .in5(products_array[5][7:0]),
        .in6(products_array[6][7:0]),
        .in7(products_array[7][7:0]),
        .sum(final_sum[39:32])
    );

    AdderTree adder_tree_5 (
        .in0(products_array[8][7:0]),
        .in1(products_array[9][7:0]),
        .in2(products_array[10][7:0]),
        .in3(products_array[11][7:0]),
        .in4(products_array[12][7:0]),
        .in5(products_array[13][7:0]),
        .in6(products_array[14][7:0]),
        .in7(products_array[15][7:0]),
        .sum(final_sum[47:40])
    );

    AdderTree adder_tree_6 (
        .in0(products_array[0][15:8]),
        .in1(products_array[1][15:8]),
        .in2(products_array[2][15:8]),
        .in3(products_array[3][15:8]),
        .in4(products_array[4][15:8]),
        .in5(products_array[5][15:8]),
        .in6(products_array[6][15:8]),
        .in7(products_array[7][15:8]),
        .sum(final_sum[55:48])
    );

    AdderTree adder_tree_7 (
        .in0(products_array[8][15:8]),
        .in1(products_array[9][15:8]),
        .in2(products_array[10][15:8]),
        .in3(products_array[11][15:8]),
        .in4(products_array[12][15:8]),
        .in5(products_array[13][15:8]),
        .in6(products_array[14][15:8]),
        .in7(products_array[15][15:8]),
        .sum(final_sum[63:56])
    );

    AdderTree adder_tree_8 (
        .in0(products_array[0][7:0]),
        .in1(products_array[1][7:0]),
        .in2(products_array[2][7:0]),
        .in3(products_array[3][7:0]),
        .in4(products_array[4][7:0]),
        .in5(products_array[5][7:0]),
        .in6(products_array[6][7:0]),
        .in7(products_array[7][7:0]),
        .sum(final_sum[71:64])
    );

    AdderTree adder_tree_9 (
        .in0(products_array[8][7:0]),
        .in1(products_array[9][7:0]),
        .in2(products_array[10][7:0]),
        .in3(products_array[11][7:0]),
        .in4(products_array[12][7:0]),
        .in5(products_array[13][7:0]),
        .in6(products_array[14][7:0]),
        .in7(products_array[15][7:0]),
        .sum(final_sum[79:72])
    );

    AdderTree adder_tree_10 (
        .in0(products_array[0][15:8]),
        .in1(products_array[1][15:8]),
        .in2(products_array[2][15:8]),
        .in3(products_array[3][15:8]),
        .in4(products_array[4][15:8]),
        .in5(products_array[5][15:8]),
        .in6(products_array[6][15:8]),
        .in7(products_array[7][15:8]),
        .sum(final_sum[87:80])
    );

    AdderTree adder_tree_11 (
        .in0(products_array[8][15:8]),
        .in1(products_array[9][15:8]),
        .in2(products_array[10][15:8]),
        .in3(products_array[11][15:8]),
        .in4(products_array[12][15:8]),
        .in5(products_array[13][15:8]),
        .in6(products_array[14][15:8]),
        .in7(products_array[15][15:8]),
        .sum(final_sum[95:88])
    );

    AdderTree adder_tree_12 (
        .in0(products_array[0][7:0]),
        .in1(products_array[1][7:0]),
        .in2(products_array[2][7:0]),
        .in3(products_array[3][7:0]),
        .in4(products_array[4][7:0]),
        .in5(products_array[5][7:0]),
        .in6(products_array[6][7:0]),
        .in7(products_array[7][7:0]),
        .sum(final_sum[103:96])
    );

    AdderTree adder_tree_13 (
        .in0(products_array[8][7:0]),
        .in1(products_array[9][7:0]),
        .in2(products_array[10][7:0]),
        .in3(products_array[11][7:0]),
        .in4(products_array[12][7:0]),
        .in5(products_array[13][7:0]),
        .in6(products_array[14][7:0]),
        .in7(products_array[15][7:0]),
        .sum(final_sum[111:104])
    );

    AdderTree adder_tree_14 (
        .in0(products_array[0][15:8]),
        .in1(products_array[1][15:8]),
        .in2(products_array[2][15:8]),
        .in3(products_array[3][15:8]),
        .in4(products_array[4][15:8]),
        .in5(products_array[5][15:8]),
        .in6(products_array[6][15:8]),
        .in7(products_array[7][15:8]),
        .sum(final_sum[119:112])
    );

    AdderTree adder_tree_15 (
        .in0(products_array[8][15:8]),
        .in1(products_array[9][15:8]),
        .in2(products_array[10][15:8]),
        .in3(products_array[11][15:8]),
        .in4(products_array[12][15:8]),
        .in5(products_array[13][15:8]),
        .in6(products_array[14][15:8]),
        .in7(products_array[15][15:8]),
        .sum(final_sum[127:120])
    );

endmodule

module WeightBuffer(
    input wire clk,
    input wire reset,
    input wire [7:0] weight_data,
    input wire write_enable,
    input wire [3:0] write_addr,
    output wire [127:0] weight_out  // Flattened array for weight data (16 x 8-bit)
);
    reg [7:0] weights [15:0];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1) begin
                weights[i] <= 8'b0;
            end
        end else if (write_enable) begin
            weights[write_addr] <= weight_data;
        end
    end

    // Flatten the output array
    assign weight_out = {weights[15], weights[14], weights[13], weights[12],
                         weights[11], weights[10], weights[9], weights[8],
                         weights[7], weights[6], weights[5], weights[4],
                         weights[3], weights[2], weights[1], weights[0]};
endmodule

module LineBuffer(
    input wire clk,
    input wire reset,
    input wire [7:0] data_in,
    input wire write_enable,
    input wire [3:0] write_addr,
    input wire [3:0] read_addr,
    output wire [127:0] data_out  // Flattened array for line data (16 x 8-bit)
);
    reg [7:0] line_data [15:0];
    integer i;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1) begin
                line_data[i] <= 8'b0;
            end
        end else if (write_enable) begin
            line_data[write_addr] <= data_in;
        end
    end
    
    // Flatten the output array
    assign data_out = {line_data[15], line_data[14], line_data[13], line_data[12],
                       line_data[11], line_data[10], line_data[9], line_data[8],
                       line_data[7], line_data[6], line_data[5], line_data[4],
                       line_data[3], line_data[2], line_data[1], line_data[0]};
endmodule

module MAC(
    input wire [7:0] a,
    input wire [7:0] b,
    output wire [15:0] product
);
    assign product = a * b;
endmodule

module MACArray(
    input wire [127:0] line_data,  // Flattened array for line data (16 x 8-bit)
    input wire [127:0] weight_data,  // Flattened array for weight data (16 x 8-bit)
    output wire [255:0] products  // Flattened array for products (16 x 16-bit)
);
    wire [7:0] line_data_array [15:0];
    wire [7:0] weight_data_array [15:0];
    wire [15:0] products_array [15:0];

    // Unflatten the input arrays
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : input_unflatten
            assign line_data_array[i] = line_data[i*8 +: 8];
            assign weight_data_array[i] = weight_data[i*8 +: 8];
        end
    endgenerate

    // Instantiate the MAC units
    generate
        for (i = 0; i < 16; i = i + 1) begin : mac
            MAC mac_unit (
                .a(line_data_array[i]),
                .b(weight_data_array[i]),
                .product(products_array[i])
            );
        end
    endgenerate

    // Flatten the output array
    generate
        for (i = 0; i < 16; i = i + 1) begin : output_flatten
            assign products[i*16 +: 16] = products_array[i];
        end
    endgenerate
endmodule


`default_nettype none



`default_nettype wire
