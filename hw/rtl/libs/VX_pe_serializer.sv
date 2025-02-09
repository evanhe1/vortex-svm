// Copyright © 2019-2023
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

`include "VX_platform.vh"

`TRACING_OFF
module VX_pe_serializer #(
    parameter NUM_LANES      = 1, 
    parameter NUM_PES        = 1, 
    parameter LATENCY        = 1,
    parameter DATA_IN_WIDTH  = 1,
    parameter DATA_OUT_WIDTH = 1,
    parameter TAG_WIDTH      = 0,
    parameter PE_REG         = 0
) (
    input wire                          clk,
    input wire                          reset,

    // input
    input wire                          valid_in,
    input wire [NUM_LANES-1:0][DATA_IN_WIDTH-1:0] data_in,    
    input wire [TAG_WIDTH-1:0]          tag_in,
    output wire                         ready_in,

    // PE
    output wire                         pe_enable,    
    output wire [NUM_PES-1:0][DATA_IN_WIDTH-1:0] pe_data_in,
    input wire [NUM_PES-1:0][DATA_OUT_WIDTH-1:0] pe_data_out,

    // output
    output wire                         valid_out,
    output wire [NUM_LANES-1:0][DATA_OUT_WIDTH-1:0] data_out,
    output wire [TAG_WIDTH-1:0]         tag_out,
    input wire                          ready_out
);
    wire valid_out_s;
    wire [TAG_WIDTH-1:0] tag_out_s;
    wire enable;

    VX_shift_register #(
        .DATAW  (1 + TAG_WIDTH),
        .DEPTH  (LATENCY + PE_REG),
        .RESETW (1)
    ) shift_reg (
        .clk      (clk),
        .reset    (reset),
        .enable   (enable),
        .data_in  ({valid_in, tag_in}),
        .data_out ({valid_out_s, tag_out_s})
    );

    if (NUM_LANES != NUM_PES) begin

        localparam BATCH_SIZE = NUM_LANES / NUM_PES;
        localparam BATCH_SIZEW = `LOG2UP(BATCH_SIZE);

        reg [BATCH_SIZEW-1:0] batch_in_idx;
        reg [BATCH_SIZEW-1:0] batch_out_idx;

        always @(posedge clk) begin
            if (reset) begin
                batch_in_idx <= '0;
                batch_out_idx <= '0;
            end else if (enable) begin
                if (valid_in) begin
                    batch_in_idx <= batch_in_idx + BATCH_SIZEW'(1);
                end
                if (valid_out_s) begin
                    batch_out_idx <= batch_out_idx + BATCH_SIZEW'(1);
                end
            end
        end

        wire batch_in_done = (batch_in_idx == BATCH_SIZEW'(BATCH_SIZE-1)); 
        wire batch_out_done = (batch_out_idx == BATCH_SIZEW'(BATCH_SIZE-1));

        wire [NUM_PES-1:0][DATA_IN_WIDTH-1:0] pe_data_in_s;
        for (genvar i = 0; i < NUM_PES; ++i) begin
            assign pe_data_in_s[i] = data_in[batch_in_idx * NUM_PES + i];
        end

        VX_pipe_register #(
            .DATAW (NUM_PES * DATA_IN_WIDTH),
            .DEPTH (PE_REG)
        ) pe_reg (
            .clk      (clk),
            .reset    (reset),
            .enable   (enable),
            .data_in  (pe_data_in_s),
            .data_out (pe_data_in)
        );

        reg valid_out_r;
        reg [BATCH_SIZE-1:0][NUM_PES-1:0][DATA_OUT_WIDTH-1:0] data_out_r;
        reg [TAG_WIDTH-1:0] tag_out_r;

        wire valid_out_b = valid_out_s && batch_out_done;        
        wire enable_r = ready_out || ~valid_out;

        always @(posedge clk) begin
            if (reset) begin
                valid_out_r <= 1'b0;
            end else if (enable_r) begin
                valid_out_r <= valid_out_b;
            end
            if (enable_r) begin            
                data_out_r[batch_out_idx] <= pe_data_out;
                tag_out_r <= tag_out_s;
            end
        end
        
        assign enable    = (enable_r || ~valid_out_b);
        assign ready_in  = enable && batch_in_done;

        assign pe_enable = enable;

        assign valid_out = valid_out_r;
        assign data_out  = data_out_r;
        assign tag_out   = tag_out_r;

    end else begin

        assign enable    = ready_out || ~valid_out;
        assign ready_in  = enable;

        assign pe_enable = enable;
        assign pe_data_in= data_in;        

        assign valid_out = valid_out_s;        
        assign data_out  = pe_data_out;
        assign tag_out   = tag_out_s;
        
    end

endmodule
`TRACING_ON
