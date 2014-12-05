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
      output reg [SRAM_ADDR_WIDTH:0]      wr_0_addr,
      output reg [DATA_WIDTH-1:0]         wr_0_data,
      input                               wr_0_ack,

      output reg                          rd_1_req,
      output reg                          rd_0_req,
      output reg [SRAM_ADDR_WIDTH:0]      rd_0_addr,
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

   localparam WAIT = 0;
   localparam WAITDATASHIFT = 1;
   localparam WAITACKSHIFT = 2;
   localparam WAITWRSHIFT = 3;
   localparam WAITBUSCABF = 4;
   localparam WAITACK = 5;
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
   reg                     in_fifo_rd_en;
   wire [21:0]             in_fifo_data_dout;
   wire                    in_wr_fifo;

   reg [21:0]              timer;
   reg                     shift_req1,shift_req2,shift_req3,shift_req4,shift_req5, shift_req6;
   reg                     shift_req1_next,shift_req2_next,shift_req3_next,shift_req4_next,shift_req5_next, shift_req6_next;
   reg                     shift_req_data1,shift_req_data2,shift_req_data3,shift_req_data4,shift_req_data5, shift_req_data6;
   reg                     shift_req_data1_next,shift_req_data2_next,shift_req_data3_next,shift_req_data4_next,shift_req_data5_next, shift_req_data6_next;
   reg                     shift_req_ack1,shift_req_ack2,shift_req_ack3,shift_req_ack4,shift_req_ack5, shift_req_ack6;
   reg                     shift_req_ack1_next,shift_req_ack2_next,shift_req_ack3_next,shift_req_ack4_next,shift_req_ack5_next, shift_req_ack6_next;
   reg                     data_pkt1,data_pkt2,data_pkt3,data_pkt4,data_pkt5,data_pkt6;
   reg                     data_pkt1_next,data_pkt2_next,data_pkt3_next,data_pkt4_next,data_pkt5_next,data_pkt6_next;
   reg                     ack_pkt1,ack_pkt2,ack_pkt3,ack_pkt4,ack_pkt5,ack_pkt6;
   reg                     ack_pkt1_next,ack_pkt2_next,ack_pkt3_next,ack_pkt4_next,ack_pkt5_next,ack_pkt6_next;
   reg [3:0]               numshift;
   reg [7:0]               bfcur;
   reg                     data_proc, ack_proc;
   reg                     data_proc_next, ack_proc_next;
   reg [2:0]               state, state_next;
   reg [6:0]               index, index_next;
   reg                     rd_0_req_next,wr_0_req_next;
   reg [SRAM_ADDR_WIDTH-1:0]  rd_0_addr_next,next_addr_next,hold_addr_next;
   reg [SRAM_ADDR_WIDTH-1:0]  next_addr,hold_addr;
   reg [SRAM_ADDR_WIDTH-1:0]  wr_0_addr_next,wr_0_addr1_next,wr_0_addr2_next,wr_0_addr3_next,wr_0_addr4_next,wr_0_addr5_next,wr_0_addr6_next;
   reg [SRAM_ADDR_WIDTH-1:0]  wr_0_addr1,wr_0_addr2,wr_0_addr3,wr_0_addr4,wr_0_addr5,wr_0_addr6;
   wire [SRAM_ADDR_WIDTH-1:0]  hash0, hash1;
   reg [DATA_WIDTH-1:0]        wr_0_data_next,data_shift_next,ack_shift_next;
   wire [DATA_WIDTH-1:0]       wr_data_shuffled;
   reg [DATA_WIDTH-1:0]        data_shift,ack_shift;

   assign in_wr_fifo = !in_fifo_nearly_full && (data_pkt||ack_pkt);
   //assign in_wr_fifo = 0;
   assign wr_data_shuffled[71:24]=(rd_0_vld&&(bfcur>=rd_0_data[23:16]))?{rd_0_data[71:24]>>(bfcur-rd_0_data[23:16])+4'b1101}:rd_0_vld?(rd_0_data[71:24]>>(12-rd_0_data[23:16]+bfcur)+4'b1001):wr_data_shuffled;
   assign wr_data_shuffled[23:0] = {bfcur,16'b0};

   fallthrough_small_fifo #(.WIDTH(22), .MAX_DEPTH_BITS(3)) input_fifo_req 
        (.din ({data_pkt,ack_pkt,hash0,hash1}),     // Data in
         .wr_en (in_wr_fifo),               // Write enable
         .rd_en (in_fifo_rd_en),       // Read the next word 
         .dout ({in_fifo_data_dout}),
         .full (),
         .nearly_full (in_fifo_nearly_full),
         .empty (in_fifo_empty),
         .reset (reset),
         .clk (clk));

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
      //$display("fifo_data_dout: %h",in_fifo_data_dout);
      $display("data_in: %h",{data_pkt,ack_pkt,hash0,hash1});
      if(rd_0_vld)
         $display("dadoslidos: %h",rd_0_data);
      /*else if(rd_0_ack)
         $display("dadosvldlidos: %h",rd_0_data);*/
      if(timer >= 22'b1110010011100001110000) begin
         timer <= 0;
         bfcur <= bfcur + 1;
         /*if(bfcur >= 12)
            bfcur <= 0;*/
      end
      else
         timer <= timer+1;
      
   end

//initialilly bloom filter was implemented with only 1 hash

   always@* begin
      if(reset) begin
         $display("RESETED\n");
         numshift = 0;
         timer = 0;
         wr_0_req = 0;
         rd_0_req = 0;
         in_fifo_rd_en = 0;
         {shift_req1,shift_req2,shift_req3,shift_req4,shift_req5,shift_req6}=6'b0;
         {data_pkt1,data_pkt2,data_pkt3,data_pkt4,data_pkt5,data_pkt6}=6'b0;
         {ack_pkt1,ack_pkt2,ack_pkt3,ack_pkt4,ack_pkt5,ack_pkt6}=6'b0;
         next_addr = 0;
         next_addr_next = 0;
         bfcur = 0;
         state = WAIT;
      end
      else begin
         /*in_fifo_rd_en = 0;
         if(!in_fifo_empty) begin
            in_fifo_rd_en = lock?0:1;
         end*/
      end
   end

   always @(posedge clk) begin
      if(rd_0_vld)
         $display("Dadoslidos: %x, shq1: %x, shq2: %x, dat3: %x, dat2: %x, wr_req: %x,ack1: %x, ack2: %x, ack3: %x, ack4: %x, ack5: %x\n",rd_0_data,shift_req1,shift_req2,data_pkt1,data_pkt2,wr_0_req,ack_pkt1,ack_pkt2,ack_pkt3,ack_pkt4,ack_pkt5);
   end

   always @(*) begin
      in_fifo_rd_en = 0;

      state_next = state;
      next_addr_next = next_addr;
      index_next = index;
      hold_addr_next = hold_addr;
      rd_0_req_next = 0;
      wr_0_req_next = data_pkt1 || ack_pkt1 || shift_req1;

      data_proc_next = 0;
      ack_proc_next = 0;

      shift_req_ack6_next = 0;
      shift_req_ack5_next = shift_req_ack6;
      shift_req_ack4_next = shift_req_ack5;
      shift_req_ack3_next = shift_req_ack4;
      shift_req_ack2_next = shift_req_ack3;
      shift_req_ack1_next = shift_req_ack2;

      shift_req_data6_next = 0;
      shift_req_data5_next = shift_req_data6;
      shift_req_data4_next = shift_req_data5;
      shift_req_data3_next = shift_req_data4;
      shift_req_data2_next = shift_req_data3;
      shift_req_data1_next = shift_req_data2;

      shift_req6_next = 0;
      shift_req5_next = shift_req6;
      shift_req4_next = shift_req5;
      shift_req3_next = shift_req4;
      shift_req2_next = shift_req3;
      shift_req1_next = shift_req2;

      data_pkt6_next = 0;
      data_pkt5_next = data_pkt6;
      data_pkt4_next = data_pkt5;
      data_pkt3_next = data_pkt4;
      data_pkt2_next = data_pkt3;
      data_pkt1_next = data_pkt2;

      ack_pkt6_next = 0;
      ack_pkt5_next = ack_pkt6;
      ack_pkt4_next = ack_pkt5;
      ack_pkt3_next = ack_pkt4;
      ack_pkt2_next = ack_pkt3;
      ack_pkt1_next = ack_pkt2;

      wr_0_addr6_next = 0;
      wr_0_addr5_next = wr_0_addr6;
      wr_0_addr4_next = wr_0_addr5;
      wr_0_addr3_next = wr_0_addr4;
      wr_0_addr2_next = wr_0_addr3;
      wr_0_addr1_next = wr_0_addr2;
      wr_0_addr_next = wr_0_addr1;

      case(state) 
         WAIT: begin
            if (!in_fifo_empty) begin
               $display("WAITIF");
               in_fifo_rd_en = 1;
               rd_0_addr_next = in_fifo_data_dout[19:10];
               wr_0_addr6_next = in_fifo_data_dout[19:10];
               rd_0_req_next = 1;
               {data_proc_next,ack_proc_next} = {in_fifo_data_dout[21:20]};
               if(in_fifo_data_dout[21]) begin //data
                  if(in_fifo_data_dout[19:10] < next_addr) begin
                     shift_req_data6_next = 0;
                     shift_req_ack6_next = 0;
                     {data_pkt6_next,ack_pkt6_next} = {in_fifo_data_dout[21:20]};
                     hold_addr_next = 0;
                     state_next = WAIT;
                  end
                  else begin
                     shift_req_data6_next = 1;
                     shift_req_ack6_next = 0;
                     {data_pkt6_next,ack_pkt6_next} = {2'b0};
                     hold_addr_next = in_fifo_data_dout[19:10];
                     state_next = WAITDATASHIFT;
                  end
               end
               else if(in_fifo_data_dout[20]) begin //ack
                  if(in_fifo_data_dout[19:10] < next_addr) begin
                     shift_req_data6_next = 0;
                     shift_req_ack6_next = 0;
                     {data_pkt6_next,ack_pkt6_next} = {in_fifo_data_dout[21:20]};
                     hold_addr_next = 0;
                     state_next = WAITACK;
                  end
                  else begin
                     shift_req_data6_next = 0;
                     shift_req_ack6_next = 1;
                     {data_pkt6_next,ack_pkt6_next} = {2'b0};
                     hold_addr_next = in_fifo_data_dout[19:10];
                     state_next = WAITACKSHIFT;
                  end
               end
            end
            else begin
               $display("WAITELSE");
               //in_fifo_empty, shifting
               //in_fifo_rd_en = 1; //not read when fifo is empty
               rd_0_addr_next = next_addr;
               rd_0_req_next = 1;
               wr_0_addr6_next = next_addr;
               shift_req6_next = 1;
               next_addr_next = next_addr+1;
               state_next = WAIT;
            end
         end
         WAITDATASHIFT: begin
            if(shift_req_data2) begin
               if(bfcur >= rd_0_data[23:16]) begin
                  data_shift_next = {rd_0_data[71:24]>>(bfcur-rd_0_data[23:16])};
               end
               else begin //12 bloomfilters
                  data_shift_next = {rd_0_data[71:24]>>(bfcur+(12-rd_0_data[23:16]))};
               end
            end
            else if(shift_req_data1) begin
               //in_fifo_rd_en = 1; //libera 
               {data_pkt1_next,ack_pkt6_next} = {2'b10};
               wr_0_data_next = {data_shift[71:68]+1'b1,data_shift[67:0]};
               wr_0_addr_next = hold_addr; 
               wr_0_req_next = 1; //force write operation
               //state_next = WAITWRSHIFT;
               state_next = WAIT;
            end
            else begin
               state_next = WAITDATASHIFT;
            end
         end
         WAITACKSHIFT: begin
            if(shift_req_ack2) begin
               if(bfcur >= rd_0_data[23:16]) begin
                  ack_shift_next = {rd_0_data[71:24]>>(bfcur-rd_0_data[23:16])};
               end
               else begin
                  ack_shift_next = {rd_0_data[71:24]>>(bfcur+(12-rd_0_data[23:16]))};
               end
               state_next = WAITBUSCABF;
            end
            else begin
               state_next = WAITACKSHIFT;
            end
         end
         WAITBUSCABF: begin
            index_next = buscaff(ack_shift);
            state_next = WAITACK;
         end
         WAITACK: begin
            wr_0_data_next[71:68] = (index[11])?ack_shift[71:68]-4'b1:ack_shift[71:68];
            wr_0_data_next[67:64] = (index[10])?ack_shift[67:64]-4'b1:ack_shift[67:64];
            wr_0_data_next[63:60] = (index[9])?ack_shift[63:60]-4'b1:ack_shift[63:60];
            wr_0_data_next[59:56] = (index[8])?ack_shift[59:56]-4'b1:ack_shift[59:56];
            wr_0_data_next[55:52] = (index[7])?ack_shift[55:52]-4'b1:ack_shift[55:52];
            wr_0_data_next[51:48] = (index[6])?ack_shift[51:48]-4'b1:ack_shift[51:48];
            wr_0_data_next[47:44] = (index[5])?ack_shift[47:44]-4'b1:ack_shift[57:44];
            wr_0_data_next[43:40] = (index[4])?ack_shift[43:40]-4'b1:ack_shift[43:40];
            wr_0_data_next[39:36] = (index[3])?ack_shift[39:36]-4'b1:ack_shift[39:36];
            wr_0_data_next[35:32] = (index[2])?ack_shift[35:32]-4'b1:ack_shift[35:32];
            wr_0_data_next[32:28] = (index[1])?ack_shift[32:28]-4'b1:ack_shift[38:22];
            wr_0_data_next[71:24] = (index[0])?ack_shift[27:24]-4'b1:ack_shift[38:22];
            wr_0_data_next[23:0] = {bfcur,16'b0};
            wr_0_addr_next = hold_addr;
            wr_0_req_next = 1; //force write operation
            //state_next = WAITWRSHIFT;
            state_next = WAIT;
         end
         WAITWRSHIFT: begin //do nothing
            //in_fifo_rd_en = 1;
            wr_0_req_next = 0; 
            state_next = WAIT;
         end
      endcase
   end

   always@(posedge clk) begin
      $display("naddr: %x,state: %x,shuffled: %x\n",next_addr_next,state_next,wr_data_shuffled);
      state <= state_next;
      index <= index_next;
      hold_addr <= hold_addr_next;
      next_addr <= next_addr_next;

      rd_0_req <= rd_0_req_next;
      rd_0_addr <= rd_0_addr_next;

      data_proc <= data_proc_next;
      ack_proc <= ack_proc_next;

      shift_req6 <= shift_req6_next;
      wr_0_addr6 <= wr_0_addr6_next;
      {data_pkt6,ack_pkt6} <= {data_pkt6_next,ack_pkt6_next};
      {shift_req_data6,shift_req_ack6} <= {shift_req_data6_next,shift_req_ack6_next};
      //pipeline 0
      shift_req5 <= shift_req5_next;
      wr_0_addr5 <= wr_0_addr5_next;
      {data_pkt5,ack_pkt5} <= {data_pkt5_next,ack_pkt5_next};
      {shift_req_data5,shift_req_ack5} <= {shift_req_data5_next,shift_req_ack5_next};
      //pipeline 1
      shift_req4 <= shift_req4_next;
      wr_0_addr4 <= wr_0_addr4_next;
      {data_pkt4,ack_pkt4} <= {data_pkt4_next,ack_pkt4_next};
      {shift_req_data4,shift_req_ack4} <= {shift_req_data4_next,shift_req_ack4_next};
      //pipeline 2
      shift_req3 <= shift_req3_next;
      wr_0_addr3 <= wr_0_addr3_next;
      {data_pkt3,ack_pkt3} <= {data_pkt3_next,ack_pkt3_next};
      {shift_req_data3,shift_req_ack3} <= {shift_req_data3_next,shift_req_ack3_next};
      //pipeline 3
      shift_req2 <= shift_req2_next;
      wr_0_addr2 <= wr_0_addr2_next;
      {data_pkt2,ack_pkt2} <= {data_pkt2_next,ack_pkt2_next};
      {shift_req_data2,shift_req_ack2} <= {shift_req_data2_next,shift_req_ack2_next};
      //pipeline 4
      shift_req1 <= shift_req1_next;
      wr_0_addr1 <= wr_0_addr1_next;
      {data_pkt1,ack_pkt1} <= {data_pkt1_next,ack_pkt1_next};
      {shift_req_data1,shift_req_ack1} <= {shift_req_data1_next,shift_req_ack1_next};
      //pipeline 5
      //wr_0_req <= data_pkt1||ack_pkt1||shift_req1;
      wr_0_req <= wr_0_req_next;
      wr_0_data <= shift_req1?wr_data_shuffled:wr_0_data_next;
      wr_0_addr <= wr_0_addr_next;
         
   end //always
endmodule
