//
// PCILeech FPGA.
//
// Top module for the Enigma X1 Artix-7 board.
//
// (c) Ulf Frisk, 2019-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_enigma_x1_top #(
    // DEVICE IDs as follows:
    // 0 = SP605, 1 = PCIeScreamer R1, 2 = AC701, 3 = PCIeScreamer R2, 4 = Screamer, 5 = NeTV2
    parameter       PARAM_DEVICE_ID = 9,
    parameter       PARAM_VERSION_NUMBER_MAJOR = 4,
    parameter       PARAM_VERSION_NUMBER_MINOR = 13,
    parameter       PARAM_CUSTOM_VALUE = 32'hffffffff,
    parameter       PARAM_UDP_STATIC_ADDR = 32'hc0a800de,   // 192.168.0.222
    parameter       PARAM_UDP_STATIC_FORCE = 1'b0,
    parameter       PARAM_UDP_PORT = 16'h6f3a               // 28474	
) (
    // SYS
    input           clk,
    input           ft601_clk,
    
    // SYSTEM LEDs and BUTTONs
    output          user_ld1_n,
    output          user_ld2_n,
    input           user_sw1_n,
    input           user_sw2_n,
    
    // PCI-E FABRIC
    output  [0:0]   pcie_tx_p,
    output  [0:0]   pcie_tx_n,
    input   [0:0]   pcie_rx_p,
    input   [0:0]   pcie_rx_n,
    input           pcie_clk_p,
    input           pcie_clk_n,
    input           pcie_present,
    input           pcie_perst_n,
    output reg      pcie_wake_n = 1'b1,
    
    // TO/FROM FT601 PADS
    output          ft601_rst_n,
    
    inout   [31:0]  ft601_data,
    output  [3:0]   ft601_be,
    input           ft601_rxf_n,
    input           ft601_txe_n,
    output          ft601_wr_n,
    output          ft601_siwu_n,
    output          ft601_rd_n,
    output          ft601_oe_n

    // ETH
    output          eth_clk50,
    output          eth_rst_n,
    input   [1:0]   eth_rx_data,
    input           eth_crs_dv,
    output          eth_tx_en,
    output  [1:0]   eth_tx_data,
    output          eth_mdc,
    inout           eth_mdio,
    input           eth_rx_err
    );
    
    // SYS
    wire            rst;
    
    // FIFO CTL <--> COM CTL
    wire [63:0]     com_dout;
    wire            com_dout_valid;
    wire [255:0]    com_din;
    wire            com_din_wr_en;
    wire            com_din_ready;
    wire            led_com;
    wire            led_pcie;
    
    // FIFO CTL <--> COM CTL
    IfComToFifo     dcom_fifo();
	
    // FIFO CTL <--> PCIe
    IfPCIeFifoCfg   dcfg();
    IfPCIeFifoTlp   dtlp();
    IfPCIeFifoCore  dpcie();
    IfShadow2Fifo   dshadow2fifo();
	
    // ----------------------------------------------------
    // TickCount64 CLK
    // ----------------------------------------------------

    time tickcount64 = 0;
    time tickcount64_reload = 0;
    always @ ( posedge clk ) begin
        tickcount64         <= user_sw2_n ? (tickcount64 + 1) : 0;
        tickcount64_reload  <= user_sw2_n ? 0 : (tickcount64_reload + 1);
    end

    assign rst = ~user_sw2_n || ((tickcount64 < 64) ? 1'b1 : 1'b0);
    assign ft601_rst_n = ~rst;
    wire led_pwronblink = ~user_sw1_n ^ (tickcount64[24] & (tickcount64[63:27] == 0));
    
    OBUF led_ld1_obuf(.O(user_ld1_n), .I(~led_pcie));
    OBUF led_ld2_obuf(.O(user_ld2_n), .I(~led_com));
    
    // ----------------------------------------------------
    // BUFFERED COMMUNICATION DEVICE (FT601)
    // ----------------------------------------------------
    
    pcileech_com i_pcileech_com (
        // SYS
        .clk                ( clk                   ),
        .clk_com            ( ft601_clk             ),
        .rst                ( rst                   ),
        .led_state_txdata   ( led_com               ),  // ->
        .led_state_invert   ( led_pwronblink        ),  // <-
        // FIFO CTL <--> COM CTL
        .dfifo              ( dcom_fifo.mp_com      ),
        // TO/FROM FT601 PADS
        .ft601_data         ( ft601_data            ),  // <> [31:0]
        .ft601_be           ( ft601_be              ),  // -> [3:0]
        .ft601_txe_n        ( ft601_txe_n           ),  // <-
        .ft601_rxf_n        ( ft601_rxf_n           ),  // <-
        .ft601_siwu_n       ( ft601_siwu_n          ),  // ->
        .ft601_wr_n         ( ft601_wr_n            ),  // ->
        .ft601_rd_n         ( ft601_rd_n            ),  // ->
        .ft601_oe_n         ( ft601_oe_n            )   // ->
        // MAC/RMII
        .eth_clk50          ( eth_clk50             ),
        .eth_rst_n          ( eth_rst_n             ),
        .eth_crs_dv         ( eth_crs_dv            ),
        .eth_rx_data        ( eth_rx_data           ),
        .eth_rx_err         ( eth_rx_err            ),
        .eth_tx_en          ( eth_tx_en             ),
        .eth_tx_data        ( eth_tx_data           ),
        .eth_mdc            ( eth_mdc               ),
        .eth_mdio           ( eth_mdio              ),
        .eth_cfg_static_addr ( PARAM_UDP_STATIC_ADDR    ),  // <- [31:0]
        .eth_cfg_static_force ( PARAM_UDP_STATIC_FORCE  ),  // <-
        .eth_cfg_port       ( PARAM_UDP_PORT        ),  // <- [15:0]
        .eth_led_state_red  ( led20                 ),  // ->
        .eth_led_state_green( led21                 )   // ->	    
    );
    
    // ----------------------------------------------------
    // FIFO CTL
    // ----------------------------------------------------
    
    pcileech_fifo #(
        .PARAM_DEVICE_ID            ( PARAM_DEVICE_ID               ),
        .PARAM_VERSION_NUMBER_MAJOR ( PARAM_VERSION_NUMBER_MAJOR    ),
        .PARAM_VERSION_NUMBER_MINOR ( PARAM_VERSION_NUMBER_MINOR    ),
        .PARAM_CUSTOM_VALUE         ( PARAM_CUSTOM_VALUE            )
    ) i_pcileech_fifo (
        .clk                ( clk                   ),
        .rst                ( rst                   ),
        .rst_cfg_reload     ( (tickcount64_reload > 500000000) ? 1'b1 : 1'b0 ),     // config reload after 5s button press
        .pcie_present       ( pcie_present          ),
        .pcie_perst_n       ( pcie_perst_n          ),
        // FIFO CTL <--> COM CTL
        .dcom               ( dcom_fifo.mp_fifo     ),
        // FIFO CTL <--> PCIe
        .dcfg               ( dcfg.mp_fifo          ),
        .dtlp               ( dtlp.mp_fifo          ),
        .dpcie              ( dpcie.mp_fifo         ),
        .dshadow2fifo       ( dshadow2fifo.fifo     )
    );
    
    // ----------------------------------------------------
    // PCIe
    // ----------------------------------------------------
    
    pcileech_pcie_a7 i_pcileech_pcie_a7(
        .clk_sys            ( clk                   ),
        .rst                ( rst                   ),
        // PCIe fabric
        .pcie_tx_p          ( pcie_tx_p             ),
        .pcie_tx_n          ( pcie_tx_n             ),
        .pcie_rx_p          ( pcie_rx_p             ),
        .pcie_rx_n          ( pcie_rx_n             ),
        .pcie_clk_p         ( pcie_clk_p            ),
        .pcie_clk_n         ( pcie_clk_n            ),
        .pcie_perst_n       ( pcie_perst_n          ),
        // State and Activity LEDs
        .led_state          ( led_pcie              ),
        // FIFO CTL <--> PCIe
        .dfifo_cfg          ( dcfg.mp_pcie          ),
        .dfifo_tlp          ( dtlp.mp_pcie          ),
        .dfifo_pcie         ( dpcie.mp_pcie         ),
	.dshadow2fifo       ( dshadow2fifo.shadow   ),

	// MAC/RMII
        .eth_clk50          ( eth_clk50             ),
        .eth_rst_n          ( eth_rst_n             ),
        .eth_crs_dv         ( eth_crs_dv            ),
        .eth_rx_data        ( eth_rx_data           ),
        .eth_rx_err         ( eth_rx_err            ),
        .eth_tx_en          ( eth_tx_en             ),
        .eth_tx_data        ( eth_tx_data           ),
        .eth_mdc            ( eth_mdc               ),
        .eth_mdio           ( eth_mdio              ),
        .eth_cfg_static_addr ( PARAM_UDP_STATIC_ADDR    ),  // <- [31:0]
        .eth_cfg_static_force ( PARAM_UDP_STATIC_FORCE  ),  // <-
        .eth_cfg_port       ( PARAM_UDP_PORT        ),  // <- [15:0]
        .eth_led_state_red  ( led20                 ),  // ->
        .eth_led_state_green( led21                 )   // ->	    
    );

endmodule
