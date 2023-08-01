// Copyright (C) 2019-2022, Université catholique de Louvain (UCLouvain, Belgium), University of Zürich (UZH, Switzerland),
//         Katholieke Universiteit Leuven (KU Leuven, Belgium), and Delft University of Technology (TU Delft, Netherlands).
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file except in compliance
// with the License, or, at your option, the Apache License version 2.0. You may obtain a copy of the License at
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the License is distributed on
// an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
//------------------------------------------------------------------------------
//
// "controller.v" - Controller module
// 
// Project: tinyODIN - A low-cost digital spiking neuromorphic processor adapted from ODIN.
//
// Author:  C. Frenkel, Delft University of Technology
//
// Cite/paper: C. Frenkel, M. Lefebvre, J.-D. Legat and D. Bol, "A 0.086-mm² 12.7-pJ/SOP 64k-Synapse 256-Neuron Online-Learning
//             Digital Spiking Neuromorphic Processor in 28-nm CMOS," IEEE Transactions on Biomedical Circuits and Systems,
//             vol. 13, no. 1, pp. 145-158, 2019.
//
//------------------------------------------------------------------------------


module controller #(
    parameter N = 256,
    parameter M = 8
)(    

    // Global inputs ------------------------------------------
    input  wire           CLK,
    input  wire           RST, // async rise reset
    
    // Inputs from AER ----------------------------------------
    input  wire   [M+1:0] AERIN_ADDR,
    input  wire           AERIN_REQ, // AER_in request handshake
    output reg            AERIN_ACK, // AER_in acknowledge handshake
    
    // Control interface for readback -------------------------
    // from spi_slave,
    input  wire           CTRL_READBACK_EVENT,   // to controller
    input  wire           CTRL_PROG_EVENT, // to controller
    input  wire [2*M-1:0] CTRL_SPI_ADDR, // to controller, neuron, synapse
    input  wire     [1:0] CTRL_OP_CODE, // to controller
    
    // Inputs from SPI configuration registers ----------------
    // from spi_slave
    input  wire           SPI_GATE_ACTIVITY,
    output reg            SPI_GATE_ACTIVITY_sync, // send to neuron and aer_out
    input  wire   [M-1:0] SPI_MAX_NEUR,
    
    // Inputs from scheduler ----------------------------------
    input  wire           SCHED_EMPTY,
    input  wire           SCHED_FULL,
    input  wire    [11:0] SCHED_DATA_OUT,
    
    // Input from AER output ----------------------------------
    input  wire           AEROUT_CTRL_BUSY,  // rise when send REQ to other core, fall when (get AEROUT_ACK_sync_negedge) fall
    
    // Outputs to synaptic core -------------------------------
    output reg            CTRL_SYNARRAY_WE,
    output reg            CTRL_NEURMEM_WE,
    output reg    [ 12:0] CTRL_SYNARRAY_ADDR, 
    output reg    [M-1:0] CTRL_NEURMEM_ADDR,
    output reg            CTRL_SYNARRAY_CS,  // chip select
    output reg            CTRL_NEURMEM_CS,
    
    // Outputs to neurons -------------------------------------
    output reg            CTRL_NEUR_EVENT, 
    output reg            CTRL_NEUR_TREF,
    output reg      [3:0] CTRL_NEUR_VIRTS,
    
    // Outputs to scheduler -----------------------------------
    output reg            CTRL_SCHED_POP_N,
    output reg    [M-1:0] CTRL_SCHED_ADDR,
    output reg            CTRL_SCHED_EVENT_IN,
    output reg    [  3:0] CTRL_SCHED_VIRTS,
    
    // Output to AER output -----------------------------------
    output wire           CTRL_AEROUT_POP_NEUR
);
    
	//----------------------------------------------------------------------------------
	//	PARAMETERS 
	//----------------------------------------------------------------------------------

	// FSM states 
	localparam WAIT       = 4'd0; 
    localparam W_NEUR     = 4'd1;
    localparam R_NEUR     = 4'd2;
    localparam W_SYN      = 4'd3;
    localparam R_SYN      = 4'd4;
	localparam TREF       = 4'd5; //
    localparam PUSH       = 4'd6; //
	localparam POP_NEUR   = 4'd7; //
    localparam POP_VIRT   = 4'd8; //
    localparam AER_POP    = 4'd9; //
    localparam WAIT_SPIDN = 4'd10; //
    localparam WAIT_REQDN = 4'd11; //


	//----------------------------------------------------------------------------------
	//	REGS & WIRES
	//----------------------------------------------------------------------------------
    
    reg          AERIN_REQ_sync_int, AERIN_REQ_sync;
    reg          SPI_GATE_ACTIVITY_sync_int;
    reg          CTRL_READBACK_EVENT_sync_int, CTRL_READBACK_EVENT_sync;
    reg          CTRL_PROG_EVENT_sync_int, CTRL_PROG_EVENT_sync;

    wire         tref_event, virt_event, neuron_event;
    
    reg  [ 31:0] ctrl_cnt;
    reg  [  7:0] neur_cnt;
    reg          neur_cnt_inc;
    
    reg  [  3:0] state, nextstate;
    
	//----------------------------------------------------------------------------------
	//	EVENT TYPE DECODING 
	//----------------------------------------------------------------------------------

    assign tref_event     = AERIN_ADDR[M  ];  // for leakage
    assign virt_event     = AERIN_ADDR[M+1];  // for test?
    assign neuron_event   = !tref_event && !virt_event;  // true spike event

	//----------------------------------------------------------------------------------
	//	SYNC BARRIERS FROM AER AND FROM SPI
	//----------------------------------------------------------------------------------
    // AERIN_REQ, SPI_GATE_ACTIVITY, CTRL_READBACK_EVENT, CTRL_PROG_EVENT
    always @(posedge CLK, posedge RST) begin
		if(RST) begin
			AERIN_REQ_sync_int           <= 1'b0;
			AERIN_REQ_sync	             <= 1'b0;
            SPI_GATE_ACTIVITY_sync_int   <= 1'b0;
            SPI_GATE_ACTIVITY_sync       <= 1'b0;
            CTRL_READBACK_EVENT_sync_int <= 1'b0;
            CTRL_READBACK_EVENT_sync     <= 1'b0;
            CTRL_PROG_EVENT_sync_int     <= 1'b0;
            CTRL_PROG_EVENT_sync         <= 1'b0;
		end
		else begin
			AERIN_REQ_sync_int           <= AERIN_REQ;
			AERIN_REQ_sync	             <= AERIN_REQ_sync_int;
            SPI_GATE_ACTIVITY_sync_int   <= SPI_GATE_ACTIVITY;
            SPI_GATE_ACTIVITY_sync       <= SPI_GATE_ACTIVITY_sync_int;
            CTRL_READBACK_EVENT_sync_int <= CTRL_READBACK_EVENT;
            CTRL_READBACK_EVENT_sync     <= CTRL_READBACK_EVENT_sync_int;
            CTRL_PROG_EVENT_sync_int     <= CTRL_PROG_EVENT;
            CTRL_PROG_EVENT_sync         <= CTRL_PROG_EVENT_sync_int;
		end
	end
    
	//----------------------------------------------------------------------------------
	//	CONTROL FSM , Mealy FSM
	//----------------------------------------------------------------------------------
    
    // State register
	always @(posedge CLK, posedge RST) begin
		if   (RST) state <= WAIT;
		else       state <= nextstate;
	end
    
	// Next state logic
	always @(*)
		case(state)
			WAIT 		:	if      (AEROUT_CTRL_BUSY)                                          nextstate = WAIT;
                            else if (SPI_GATE_ACTIVITY_sync) begin // gates the network; WRITE or READ neuron/synapse memory 
                                if      (CTRL_PROG_EVENT_sync     && (CTRL_OP_CODE == 2'b01))   nextstate = W_NEUR;
                                else if (CTRL_READBACK_EVENT_sync && (CTRL_OP_CODE == 2'b01))   nextstate = R_NEUR;
                                else if (CTRL_PROG_EVENT_sync     && (CTRL_OP_CODE == 2'b10))   nextstate = W_SYN;
                                else if (CTRL_READBACK_EVENT_sync && (CTRL_OP_CODE == 2'b10))   nextstate = R_SYN;
                                else                                                            nextstate = WAIT;
                            end else begin
                                if (SCHED_FULL) // fifo full
                                    if      (|SCHED_DATA_OUT[11:8])        nextstate = POP_VIRT;
                                    else                                   nextstate = POP_NEUR;
                                else if (AERIN_REQ_sync) // fifo not full
                                    if      (tref_event)                   nextstate = TREF;
                                    else if (virt_event | neuron_event)    nextstate = PUSH;
                                    else                                   nextstate = WAIT;
                                else if (~SCHED_EMPTY) // fifo not empty
                                    if      (|SCHED_DATA_OUT[11:8])        nextstate = POP_VIRT;
                                    else                                   nextstate = POP_NEUR;
                                else                                       nextstate = WAIT;
                            end
			W_NEUR    	:   if (ctrl_cnt == 32'd1 )   nextstate = WAIT_SPIDN;
							else  		              nextstate = W_NEUR;
			R_NEUR    	:                             nextstate = WAIT_SPIDN;
			W_SYN    	:   if (ctrl_cnt == 32'd1 )   nextstate = WAIT_SPIDN;
							else			          nextstate = W_SYN;
			R_SYN    	:                             nextstate = WAIT_SPIDN;
			TREF    	:   if (~&AERIN_ADDR[M-1:0] ? (ctrl_cnt == 32'd1) : ((neur_cnt == SPI_MAX_NEUR) && neur_cnt_inc))
                            // 判断是单神经元泄露(有必要？)还是全部神经元泄露
                                                      nextstate = WAIT_REQDN;
							else					  nextstate = TREF;
            PUSH        :                             nextstate = WAIT_REQDN; 
			POP_NEUR    :   if (ctrl_cnt[8:0] == {SPI_MAX_NEUR,1'b1}) 
                                                      nextstate = AER_POP;
							else					  nextstate = POP_NEUR;                
			POP_VIRT    :   if (~CTRL_SCHED_POP_N)    nextstate = WAIT; // CTRL_SCHED_POP_N=0
							else					  nextstate = POP_VIRT;
            AER_POP     :   if (!AEROUT_CTRL_BUSY)    nextstate = WAIT;
                            else                      nextstate = AER_POP;   
			WAIT_SPIDN 	:   if (~CTRL_PROG_EVENT_sync && ~CTRL_READBACK_EVENT_sync)
                                                      nextstate = WAIT;
							else	    		      nextstate = WAIT_SPIDN;
			WAIT_REQDN 	:   if (~AERIN_REQ_sync)      nextstate = WAIT;
							else					  nextstate = WAIT_REQDN;
			default		:							  nextstate = WAIT;
		endcase 
        
    // Control counter
    // non-wait state & !AEROUT_CTRL_BUSY: ctrl_cnt+=1
	always @(posedge CLK, posedge RST) begin
		if      (RST)               ctrl_cnt <= 32'd0;
        else if (state == WAIT)     ctrl_cnt <= 32'd0;
		else if (!AEROUT_CTRL_BUSY) ctrl_cnt <= ctrl_cnt + 32'd1;
        else                        ctrl_cnt <= ctrl_cnt;
    end

    // Time-multiplexed neuron counter
    // neur_cnt_inc=1 & !AEROUT_CTRL_BUSY: neur_cnt+=1
    // TREF or POP_NEUR
	always @(posedge CLK, posedge RST)
		if      (RST)                                neur_cnt <= 8'd0;
        else if (state == WAIT)                      neur_cnt <= 8'd0;
		else if (neur_cnt_inc && !AEROUT_CTRL_BUSY)  neur_cnt <= neur_cnt + 8'd1;
        else                                         neur_cnt <= neur_cnt;
        
    assign CTRL_AEROUT_POP_NEUR = (state == POP_NEUR) && (nextstate != POP_NEUR);
 
 
 
    // Output logic      
    always @(*) begin
    
        if (state == W_NEUR) begin 
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0; 
            
            CTRL_NEURMEM_ADDR   = CTRL_SPI_ADDR[M-1:0]; // low 8-bit address
            CTRL_NEURMEM_CS     = 1'b1; // neuron memory chip select
            if (ctrl_cnt == 32'd0) begin
                CTRL_NEURMEM_WE = 1'b0; // 1 cycle 
            end else begin
                CTRL_NEURMEM_WE = 1'b1; // 1 cycle, turn to WAIT_SPIDN when ctrl_cnt=2
            end
            
        end else if (state == R_NEUR) begin
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0; 
            
            CTRL_NEURMEM_ADDR   = CTRL_SPI_ADDR[M-1:0];
            CTRL_NEURMEM_CS     = 1'b1;
            CTRL_NEURMEM_WE     = 1'b0; 
            
        end else if (state == W_SYN) begin
            CTRL_NEURMEM_ADDR   = 8'b0;
            CTRL_NEURMEM_CS     = 1'b0;
            CTRL_NEURMEM_WE     = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0;  
            
            CTRL_SYNARRAY_ADDR  = CTRL_SPI_ADDR[12:0];
            CTRL_SYNARRAY_CS    = 1'b1;
            if (ctrl_cnt == 32'd0) begin
                CTRL_SYNARRAY_WE = 1'b0;
            end else begin
                CTRL_SYNARRAY_WE = 1'b1;
            end 
            
        end else if (state == R_SYN) begin  
            CTRL_NEURMEM_ADDR   = 8'b0;
            CTRL_NEURMEM_CS     = 1'b0;
            CTRL_NEURMEM_WE     = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0; 
            
            CTRL_SYNARRAY_ADDR   = CTRL_SPI_ADDR[12:0];
            CTRL_SYNARRAY_CS     = 1'b1;
            CTRL_SYNARRAY_WE     = 1'b0;
        
        // leakage
        end else if (state == TREF) begin
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            AERIN_ACK           = 1'b0;
            
            CTRL_NEURMEM_ADDR   = &AERIN_ADDR[M-1:0] ? neur_cnt : AERIN_ADDR[M-1:0];
            CTRL_NEUR_EVENT     = 1'b1;
            CTRL_NEUR_TREF      = 1'b1;
            CTRL_NEURMEM_CS     = 1'b1;
            if (ctrl_cnt[0] == 1'd0) begin
                CTRL_NEURMEM_WE = 1'b0;
                neur_cnt_inc    = 1'b0;
            end else begin
                CTRL_NEURMEM_WE = 1'b1;
                neur_cnt_inc    = &AERIN_ADDR[M-1:0]; //
            end

        // in_event: virtul or neuron event
        end else if (state == PUSH) begin
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEURMEM_ADDR   = 8'b0;
            CTRL_NEURMEM_CS     = 1'b0;
            CTRL_NEURMEM_WE     = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0;
            
            // virtual event: w[3:0]  +  4'b0000 + neur[3:0]
            // non virtual  : 4'b0000 +     neur[7:0]
            CTRL_SCHED_VIRTS    = AERIN_ADDR[M+1] ?        AERIN_ADDR[M-1:4]  :              4'b0;
            CTRL_SCHED_ADDR     = AERIN_ADDR[M+1] ? {4'h0, AERIN_ADDR[  3:0]} : AERIN_ADDR[M-1:0];
            CTRL_SCHED_EVENT_IN = 1'b1; // push 12'{CTRL_SCHED_VIRTS,CTRL_SCHED_ADDR} into fifo

        end else if (state == POP_NEUR) begin  
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            AERIN_ACK           = 1'b0;
            
            CTRL_SYNARRAY_ADDR  = {SCHED_DATA_OUT[M-1:0],neur_cnt[7:3]};
            CTRL_SYNARRAY_CS    = ~|neur_cnt[2:0];
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEURMEM_ADDR   = neur_cnt;  
            CTRL_SCHED_POP_N    = 1'b1;
            CTRL_NEUR_EVENT     = 1'b1;
            CTRL_NEURMEM_CS     = 1'b1;
            if (ctrl_cnt[0] == 1'b0) begin
                CTRL_NEURMEM_WE = 1'b0;
                neur_cnt_inc    = 1'b0;
            end else begin
                CTRL_NEURMEM_WE = 1'b1;
                neur_cnt_inc    = 1'b1;    //
            end 
        
        end else if (state == POP_VIRT) begin  
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0;
            
            CTRL_NEURMEM_ADDR   = SCHED_DATA_OUT[M-1:0];
            CTRL_NEUR_VIRTS     = SCHED_DATA_OUT[ 11:M];
            CTRL_NEUR_EVENT     = 1'b1;
            CTRL_NEURMEM_CS     = 1'b1;
            if (ctrl_cnt == 32'd0) begin
                CTRL_NEURMEM_WE  = 1'b0;
                CTRL_SCHED_POP_N = 1'b1;
            end else begin
                CTRL_NEURMEM_WE  = 1'b1;
                CTRL_SCHED_POP_N = 1'b0; //
            end
        
        end else if (state == AER_POP) begin  
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEURMEM_ADDR   = 8'b0;
            CTRL_NEURMEM_CS     = 1'b0;
            CTRL_NEURMEM_WE     = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0;

            CTRL_SCHED_POP_N    = AEROUT_CTRL_BUSY; //
        
        end else if (state == WAIT_REQDN) begin
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEURMEM_ADDR   = 8'b0;
            CTRL_NEURMEM_CS     = 1'b0;
            CTRL_NEURMEM_WE     = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            neur_cnt_inc        = 1'b0;
            
            AERIN_ACK           = 1'b1;       // AERIN_ACK=1

        end else begin
            CTRL_SYNARRAY_ADDR  = 13'b0;
            CTRL_SYNARRAY_CS    = 1'b0;
            CTRL_SYNARRAY_WE    = 1'b0;
            CTRL_NEURMEM_ADDR   = 8'b0;
            CTRL_NEURMEM_CS     = 1'b0;
            CTRL_NEURMEM_WE     = 1'b0;
            CTRL_NEUR_VIRTS     = 4'b0;
            CTRL_NEUR_EVENT     = 1'b0;
            CTRL_NEUR_TREF      = 1'b0;
            CTRL_SCHED_VIRTS    = 4'b0;
            CTRL_SCHED_ADDR     = 8'b0;
            CTRL_SCHED_EVENT_IN = 1'b0;
            CTRL_SCHED_POP_N    = 1'b1;
            AERIN_ACK           = 1'b0;
            neur_cnt_inc        = 1'b0;
        end
    end

    
endmodule

	// localparam WAIT       = 4'd0; 
    // localparam W_NEUR     = 4'd1;
    // localparam R_NEUR     = 4'd2;
    // localparam W_SYN      = 4'd3;
    // localparam R_SYN      = 4'd4;
	// localparam TREF       = 4'd5; //
    // localparam PUSH       = 4'd6; //
	// localparam POP_NEUR   = 4'd7; //
    // localparam POP_VIRT   = 4'd8; //
    // localparam AER_POP    = 4'd9; //
    // localparam WAIT_SPIDN = 4'd10; //
    // localparam WAIT_REQDN = 4'd11; //
// --state-----CTRL_SCHED_POP_N(0)---------neur_cnt_inc(1)
// --POP_VIRT---(ctrl_cnt!=0)-----------
// --AER_POP----AEROUT_CTRL_BUSY-----------
// --TREF------------1---------------AERIN_ADDR&&ctrl_cnt[0]
// --POP_NEUR--------1---------------------ctrl_cnt[0]