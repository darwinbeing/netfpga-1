`timescale  1ns /  10ps

module bloom_filter
   #(
      parameter DATA_WIDTH = 64,
      parameter CTRL_WIDTH = DATA_WIDTH/8,
      parameter SRAM_ADDR_WIDTH = 18, //created
      parameter UDP_REG_SRC_WIDTH = 2
   )
   (
      input  [DATA_WIDTH-1:0]             in_data,
      input  [CTRL_WIDTH-1:0]             in_ctrl,
      input                               in_wr,
      output                              in_rdy,

      output [DATA_WIDTH-1:0]             out_data,
      output [CTRL_WIDTH-1:0]             out_ctrl,
      output                              out_wr,
      input                               out_rdy,

      output reg                          wr_1_req,
      output reg                          wr_0_req,
      output reg [SRAM_ADDR_WIDTH-1:0]      wr_0_addr,
      output reg [DATA_WIDTH-1:0]         wr_0_data,
      input                               wr_0_ack,

      output reg                          rd_1_req,
      output reg                          rd_0_req,
      output reg [SRAM_ADDR_WIDTH-1:0]      rd_0_addr,
      input [DATA_WIDTH-1:0]              rd_0_data,
      input                               rd_0_ack,
      input                               rd_0_vld,

      // --- Register interface
      input                               reg_req_in,
      input                               reg_ack_in,
      input                               reg_rd_wr_L_in,
      input  [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_in,
      input  [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_in,
      input  [UDP_REG_SRC_WIDTH-1:0]      reg_src_in,

      output                              reg_req_out,
      output                              reg_ack_out,
      output                              reg_rd_wr_L_out,
      output  [`UDP_REG_ADDR_WIDTH-1:0]   reg_addr_out,
      output  [`CPCI_NF2_DATA_WIDTH-1:0]  reg_data_out,
      output  [UDP_REG_SRC_WIDTH-1:0]     reg_src_out,

      // misc
      input                                reset,
      input                                clk
   );

   localparam SHIFT_RD = 0;
   localparam SHIFT_WR = 1;
   localparam LE_MEM = 2;
   localparam NOP = 3;
   localparam BITSBF    = 4;

   // Define the log2 function
   `LOG2_FUNC

   function [11:0] buscaff;
      input [71:0] data;
      reg [11:0]  index;
      begin
      index[11] = data[71:71-BITSBF+1]>0;
      index[10] = (~index[11])&data[71-BITSBF:71-2*BITSBF-1]>0;
      index[9] = (~index[10]&~index[11])&data[71-2*BITSBF:71-3*BITSBF-1]>0;
      index[8] = (~index[9]&~index[10]&~index[11])&data[71-3*BITSBF:71-4*BITSBF-1]>0;
      index[7] = (~index[8]&~index[9]&~index[10]&~index[11])&data[71-4*BITSBF:71-5*BITSBF-1]>0;
      index[6] = (~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-5*BITSBF:71-6*BITSBF-1]>0;
      index[5] = (~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-6*BITSBF:71-7*BITSBF-1]>0;
      index[4] = (~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-7*BITSBF:71-8*BITSBF-1]>0;
      index[3] = (~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-8*BITSBF:71-9*BITSBF-1]>0;
      index[2] = (~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-9*BITSBF:71-10*BITSBF-1]>0;
      index[1] = (~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-10*BITSBF:71-11*BITSBF-1]>0;
      index[0] = (~index[1]&~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11])&data[71-11*BITSBF:71-12*BITSBF-1]>0;
      buscaff = index; 
      /*buscaff = {index[11],index[10]&~index[11],
         index[9]&~index[10]&~index[11],
         index[8]&~index[9]&~index[10]&~index[11],
         index[7]&~index[8]&~index[9]&~index[10]&~index[11],
         index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11],
         index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11],
         index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11],
         index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11],
         index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11],
         index[1]&~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11],
         index[0]&~index[1]&~index[2]&~index[3]&~index[4]&~index[5]&~index[6]&~index[7]&~index[8]&~index[9]&~index[10]&~index[11]};*/
   end
   endfunction

   //fifo ctrl
   wire                    in_fifo_empty;
   wire                    in_fifo_data_empty;
   wire                    in_fifo_addr_empty;

   reg                     in_fifo_rd_en;
   reg                     in_fifo_data_rd_en;
   reg                     in_fifo_addr_rd_en;

   wire [21:0]             in_fifo_dout;
   wire [71:0]             in_fifo_data_dout;
   wire [SRAM_ADDR_WIDTH-1:0]             in_fifo_addr_dout;
   wire                    in_wr_fifo;
   wire                    in_wr_data_fifo;
   wire                    in_fifo_addr_nearly_full;
   wire                    in_fifo_data_nearly_full;
   wire                    in_fifo_nearly_full;
   reg                     in_wr_addr_fifo;

   reg [21:0]              timer;
   reg [7:0]               bfcur;
   reg                     data_proc, ack_proc;
   reg                     data_proc_next, ack_proc_next;
   reg [2:0]               state, state_next;
   reg [6:0]               index, index_next;
   reg                     rd_0_req_next,wr_0_req_next;
   reg [SRAM_ADDR_WIDTH-1:0]  rd_0_addr_next, wr_0_addr_next,next_addr_next;
   reg [SRAM_ADDR_WIDTH-1:0]  next_addr,next_addr_wr,next_addr_wr_next,addr_read_next, addr_read;
   wire [SRAM_ADDR_WIDTH-1:0]  hash0, hash1;
   reg [DATA_WIDTH-1:0]        wr_0_data_next;

   reg [2:0]                iter_shread_next, iter_shread;
   assign in_wr_fifo = !in_fifo_nearly_full && (data_pkt||ack_pkt);
   //assign in_wr_fifo = 0;
   assign in_wr_data_fifo = rd_0_vld;

   fallthrough_small_fifo #(.WIDTH(22), .MAX_DEPTH_BITS(3)) input_fifo_req 
        (.din ({data_pkt,ack_pkt,hash0,hash1}),     // Data in
         .wr_en (in_wr_fifo),               // Write enable
         .rd_en (in_fifo_rd_en),       // Read the next word 
         .dout ({in_fifo_dout}),
         .full (),
         .nearly_full (in_fifo_nearly_full),
         .empty (in_fifo_empty),
         .reset (reset),
         .clk (clk));

   fallthrough_small_fifo #(.WIDTH(72), .MAX_DEPTH_BITS(3)) input_fifo_rd_data 
        (.din ({rd_0_data}),     // Data in
         .wr_en (in_wr_data_fifo),               // Write enable
         .rd_en (in_fifo_data_rd_en),       // Read the next word 
         .dout ({in_fifo_data_dout}),
         .full (),
         .nearly_full (in_fifo_data_nearly_full),
         .empty (in_fifo_data_empty),
         .reset (reset),
         .clk (clk));

/*   fallthrough_small_fifo #(.WIDTH(SRAM_ADDR_WIDTH), .MAX_DEPTH_BITS(3)) input_fifo_addr 
        (.din ({rd_0_addr_next}),     // Data in
         .wr_en (in_wr_addr_fifo),               // Write enable
         .rd_en (in_fifo_addr_rd_en),       // Read the next word 
         .dout ({in_fifo_addr_dout}),
         .full (),
         .nearly_full (in_fifo_addr_nearly_full),
         .empty (in_fifo_addr_empty),
         .reset (reset),
         .clk (clk));*/

   simulacao #(
        .DATA_WIDTH(DATA_WIDTH),
        .CTRL_WIDTH(CTRL_WIDTH),
        .UDP_REG_SRC_WIDTH (UDP_REG_SRC_WIDTH),
        .SRAM_ADDR_WIDTH(SRAM_ADDR_WIDTH) //added
    ) simulacao (
        .out_data              (out_data),
        .out_ctrl              (out_ctrl),
        .out_wr                (out_wr),
        .out_rdy               (out_rdy),

        .in_data              (/*in_fifo_data*/in_data),
        .in_ctrl              (/*in_fifo_ctrl*/in_ctrl),
        .in_wr                (in_wr),
        .in_rdy               (in_rdy),

        .reg_req_in           (reg_req_in),
        .reg_ack_in           (reg_ack_in),
        .reg_rd_wr_L_in       (reg_rd_wr_L_in),
        .reg_addr_in          (reg_addr_in),
        .reg_data_in          (reg_data_in),
        .reg_src_in           (reg_src_in),

        .reg_req_out           (reg_req_out),
        .reg_ack_out           (reg_ack_out),
        .reg_rd_wr_L_out       (reg_rd_wr_L_out),
        .reg_addr_out          (reg_addr_out),
        .reg_data_out          (reg_data_out),
        .reg_src_out           (reg_src_out),

        .hash_0               (hash0),
        .hash_1               (hash1),
        .data_pkt             (data_pkt),
        .ack_pkt              (ack_pkt),
        .data_proc            (data_proc),
        .ack_proc             (ack_proc),

        .clk              (clk),
        .reset            (reset));

   always@(posedge clk) begin
      if(data_pkt||ack_pkt)
         $display("Chegou simulacao\n");
      //if(timer >= 22'b1110010011100001110000) begin
      if(timer >= 'b11111110000) begin
         timer <= 0;
         bfcur <= bfcur + 1;
         /*if(bfcur >= 12)
            bfcur <= 0;*/
      end
      else
         timer <= timer+1;
   end

//initialilly bloom filter was implemented with only 1 hash

   always@(reset) begin
         $display("RESETED\n");
         timer = 0;
         wr_0_req = 0;
         rd_0_req = 0;
         in_fifo_rd_en = 0;
         in_fifo_data_rd_en = 0;
         in_fifo_addr_rd_en = 0;
         next_addr = 0;
         bfcur = 5;
         iter_shread = 0;
         state = SHIFT_RD;
         next_addr_wr = 0;
         addr_read = 0;
   end

   always @(posedge clk) begin
      if(rd_0_vld)
         $display("Dadoslidos: %x\n",rd_0_data);
   end

   always @(*) begin
      in_fifo_rd_en = 0;
      in_fifo_data_rd_en = 0;
      in_fifo_addr_rd_en = 0;

      state_next = state;
      next_addr_next = next_addr;
      next_addr_wr_next = next_addr_wr;
      addr_read_next = addr_read;

      rd_0_req_next = 0;
      rd_0_addr_next = 0;

      wr_0_req_next = 0;
      wr_0_addr_next = wr_0_addr;

      iter_shread_next = iter_shread;

      //test shifter we answer all reqs without processing
      {data_proc_next,ack_proc_next} = in_fifo_dout[21:20];
      
      case(state) 
         SHIFT_RD: begin
            $display("SHIFT_RD: %h\n",in_fifo_addr_nearly_full);
            if(!in_fifo_data_nearly_full) begin
               if(iter_shread > 5)
                  state_next = SHIFT_WR;
               else begin
                  rd_0_req_next = 1;
                  rd_0_addr_next = next_addr;
                  next_addr_next = next_addr+1;
                  iter_shread_next = iter_shread+1;
                  state_next = SHIFT_RD;
               end
            end
            else
               state_next = SHIFT_WR;
            //if fifo full we don't generate new reqs 
         end
         SHIFT_WR: begin
            $display("SHIFT_WR\n");
            if(!in_fifo_data_empty) begin
               in_fifo_data_rd_en = 1;
               wr_0_req_next = 1;
               //wr_0_addr_next = in_fifo_addr_dout;
               wr_0_addr_next = next_addr_wr;
               wr_0_data_next = {4'b0,in_fifo_data_dout[71:28],bfcur,{2'b0,next_addr_wr},4'b0};
               next_addr_wr_next = next_addr_wr +1;
               iter_shread_next = iter_shread-1;
               state_next = SHIFT_WR;
            end
            else begin
               if(iter_shread) begin
                  state_next = SHIFT_WR;
               end
               else begin
                  state_next = SHIFT_RD;
               end
            end
         end
         LE_MEM: begin
            if(addr_read <= 2^10-1) begin
               rd_0_addr_next = addr_read;
               rd_0_req_next = 1;
               addr_read_next = addr_read + 1; 
               state_next = LE_MEM;
            end
            else
               state_next = NOP;
         end
         NOP: begin //do nothing
            rd_0_req_next = 1;
         end
         /*ANSWER_RD: begin
            if(!in_fifo_dout) begin
               in_fifo_rd_en = 0;
               rd_0_addr_next = in_fifo_data_dout[19:10];
               wr_0_addr_next = in_fifo_data_dout[19:10];
               rd_0_req_next = 1;
               state_next = ANSWER_WR;
            end
            else
               state_next = SHIFT_RD;
         end
         ANSWER_WR: begin
            if(in_fifo_data_empty)
               state_next = ANSWER_WR;
            else begin
               wr_0_req_next = 1;
               wr_0_data_next = {in_fifo_data_dout[71:68]+4'h1,in_fifo_data_dout[67:28],bfcur,16'b0};
               state_next = SHIFT_RD;
            end*/
      endcase
   end

   always@(posedge clk) begin
      //$display("addr_empty: %b, data_empty: %b\n",in_fifo_addr_empty,in_fifo_data_empty);
      state <= state_next;
      next_addr <= next_addr_next;
      next_addr_wr <= next_addr_wr_next;
      addr_read <= addr_read_next;

      rd_0_req <= rd_0_req_next;
      rd_0_addr <= rd_0_addr_next;

      data_proc <= data_proc_next;
      ack_proc <= ack_proc_next;

      iter_shread <= iter_shread_next;

      wr_0_data <= wr_0_data_next;
      wr_0_req <= wr_0_req_next;
      wr_0_addr <= wr_0_addr_next;
   end //always
endmodule
