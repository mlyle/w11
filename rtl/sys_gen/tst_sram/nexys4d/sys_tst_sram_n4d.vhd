-- $Id: sys_tst_sram_n4.vhd 791 2016-07-21 22:01:10Z mueller $
--
-- Copyright 2013-2016 by Walter F.J. Mueller <W.F.J.Mueller@gsi.de>
--
-- This program is free software; you may redistribute and/or modify it under
-- the terms of the GNU General Public License as published by the Free
-- Software Foundation, either version 2, or at your option any later version.
--
-- This program is distributed in the hope that it will be useful, but
-- WITHOUT ANY WARRANTY, without even the implied warranty of MERCHANTABILITY
-- or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
-- for complete details.
--
------------------------------------------------------------------------------
-- Module Name:    sys_tst_sram_n4d - syn
-- Description:    test of nexys4 fake sram (ram2ddr) and its controller
--
-- Dependencies:   vlib/xlib/s7_cmt_sfs
--                 vlib/genlib/clkdivce
--                 bplib/bpgen/bp_rs232_4line_iob
--                 bplib/bpgen/sn_humanio
--                 vlib/rlink/rlink_sp2c
--                 tst_sram
--                 bplib/nxcramlib/nx_cram_memctl_as
--                 vlib/rbus/rbd_usracc
--                 vlib/rbus/rb_sres_or_2
--                 XXX
--
-- Test bench:     XXX 
--
-- Target Devices: generic
-- Tool versions:  viv 2016.4; ghdl 0.29-0.33
--
-- Synthesized:
-- Date         Rev  viv    Target       flop  lutl  lutm  bram  slic
-- XXX 
--
-- Revision History: 
-- Date         Rev Version  Comment
-- XXX
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.ALL;

use work.slvtypes.all;
use work.xlib.all;
use work.genlib.all;
use work.serportlib.all;
use work.rblib.all;
use work.rbdlib.all;
use work.rlinklib.all;
use work.bpgenlib.all;
use work.s3boardlib.all;
use work.nxcramlib.all;
use work.sys_conf.all;

-- ----------------------------------------------------------------------------

entity sys_tst_sram_n4d is              -- top level
                                        -- implements nexys4_cram_aif
  port (
    I_CLK100 : in slbit;                -- 100 MHz clock
    I_RXD : in slbit;                   -- receive data (board view)
    O_TXD : out slbit;                  -- transmit data (board view)
    O_RTS_N : out slbit;                -- rx rts (board view; act.low)
    I_CTS_N : in slbit;                 -- tx cts (board view; act.low)
    I_SWI : in slv16;                   -- n4 switches
    I_BTN : in slv5;                    -- n4 buttons
    I_BTNRST_N : in slbit;              -- n4 reset button
    O_LED : out slv16;                  -- n4 leds
    O_RGBLED0 : out slv3;               -- n4 rgb-led 0
    O_RGBLED1 : out slv3;               -- n4 rgb-led 1
    O_ANO_N : out slv8;                 -- 7 segment disp: anodes   (act.low)
    O_SEG_N : out slv8;                 -- 7 segment disp: segments (act.low)

    O_DDR2_ADDR: out slv13;             -- SDRAM address
    O_DDR2_BA: out slv3;                -- bank
    O_DDR2_RAS_N: out slbit;            -- row address strobe
    O_DDR2_CAS_N: out slbit;            -- column
    O_DDR2_WE_N: out slbit;             -- write enable
    O_DDR2_CK_P: out slv1;              -- pos clock
    O_DDR2_CK_N: out slv1;              -- inverted clock
    O_DDR2_CKE: out slv1;               -- clock enable
    O_DDR2_CS_N: out slv1;              -- chip select
    O_DDR2_DM: out slv2;                -- data mask
    O_DDR2_ODT: out slv1;               -- on die termination enable
    IO_DDR2_DQ: inout slv16;            -- data
    IO_DDR2_DQS_P: inout slv2;          -- diff data qualification strobe - pos
    IO_DDR2_DQS_N: inout slv2           -- diff data qualification strobe - negated
  );
end sys_tst_sram_n4d;

architecture syn of sys_tst_sram_n4d is
  
  component ram2ddrxadc is
   port (
      -- Common
      clk_200MHz_i         : in    std_logic; -- 200 MHz system clock
      rst_i                : in    std_logic; -- active high system reset
      device_temp_i        : in    std_logic_vector(11 downto 0);
      
      -- RAM interface
      ram_a                : in    std_logic_vector(26 downto 0);
      ram_dq_i             : in    std_logic_vector(15 downto 0);
      ram_dq_o             : out   std_logic_vector(15 downto 0);
      ram_cen              : in    std_logic;
      ram_oen              : in    std_logic;
      ram_wen              : in    std_logic;
      ram_ub               : in    std_logic;
      ram_lb               : in    std_logic;
      
      -- DDR2 interface
      ddr2_addr            : out   std_logic_vector(12 downto 0);
      ddr2_ba              : out   std_logic_vector(2 downto 0);
      ddr2_ras_n           : out   std_logic;
      ddr2_cas_n           : out   std_logic;
      ddr2_we_n            : out   std_logic;
      ddr2_ck_p            : out   std_logic_vector(0 downto 0);
      ddr2_ck_n            : out   std_logic_vector(0 downto 0);
      ddr2_cke             : out   std_logic_vector(0 downto 0);
      ddr2_cs_n            : out   std_logic_vector(0 downto 0);
      ddr2_dm              : out   std_logic_vector(1 downto 0);
      ddr2_odt             : out   std_logic_vector(0 downto 0);
      ddr2_dq              : inout std_logic_vector(15 downto 0);
      ddr2_dqs_p           : inout std_logic_vector(1 downto 0);
      ddr2_dqs_n           : inout std_logic_vector(1 downto 0)
   );
  end component;

  signal CLK :   slbit := '0';
  signal CLK200 : slbit := '0';

  signal CE_USEC :  slbit := '0';
  signal CE_MSEC :  slbit := '0';

  signal CLKS :  slbit := '0';
  signal CES_MSEC : slbit := '0';

  signal GBL_RESET : slbit := '0';
  
  signal RXD :   slbit := '1';
  signal TXD :   slbit := '0';
  signal CTS_N : slbit := '0';
  signal RTS_N : slbit := '0';

  signal SWI     : slv16 := (others=>'0');
  signal BTN     : slv5  := (others=>'0');
  signal LED     : slv16 := (others=>'0');  
  signal DSP_DAT : slv32 := (others=>'0');
  signal DSP_DP  : slv8  := (others=>'0');

  signal RB_MREQ : rb_mreq_type := rb_mreq_init;
  signal RB_SRES : rb_sres_type := rb_sres_init;
  signal RB_LAM  : slv16 := (others=>'0');
  signal RB_STAT : slv4 := (others=>'0');
  
  signal SER_MONI : serport_moni_type := serport_moni_init;

  signal RB_SRES_TST : rb_sres_type := rb_sres_init;
  signal RB_SRES_USRACC : rb_sres_type := rb_sres_init;

  signal RB_LAM_TST  : slbit := '0';

  signal MEM_RESET : slbit := '0';
  signal MEM_REQ   : slbit := '0';
  signal MEM_WE    : slbit := '0';
  signal MEM_BUSY  : slbit := '0';
  signal MEM_ACK_R : slbit := '0';
  signal MEM_ACK_W : slbit := '0';
  signal MEM_ACT_R : slbit := '0';
  signal MEM_ACT_W : slbit := '0';
  signal MEM_ADDR  : slv22 := (others=>'0');
  signal MEM_BE    : slv4  := (others=>'0');
  signal MEM_DI    : slv32 := (others=>'0');
  signal MEM_DO    : slv32 := (others=>'0');

  signal CRAM_CE_N   : slbit := '0';            -- cram: chip enable   (act.low)
  signal CRAM_BE_N   : slv2 := (others=>'0');   -- cram: byte enables  (act.low)
  signal CRAM_WE_N   : slbit := '0';            -- cram: write enable  (act.low)
  signal CRAM_OE_N   : slbit := '0';            -- cram: output enable (act.low)
  signal CRAM_ADV_N  : slbit := '0';            -- cram: address valid (act.low)
  signal CRAM_CLK    : slbit := '0';            -- cram: clock
  signal CRAM_CRE    : slbit := '0';            -- cram: command register enable
  signal CRAM_ADDR   : slv23 := (others=>'0');  -- cram: address lines
  signal CRAM_ADDR27 : std_logic_vector(26 downto 0) := (others=>'0');
  signal CRAM_DATAC  : slv16 := (others=>'0');  -- cram: data lines at controller
  signal CRAM_DATAO  : slv16 := (others=>'0');  -- cram: data line to ram2ddr
  signal CRAM_DATAI  : slv16 := (others=>'0');  -- cram: data line from ram2ddr
  
  
  constant sysid_proj  : slv16 := x"0104";   -- tst_sram
  constant sysid_board : slv8  := x"05";     -- nexys4
  constant sysid_vers  : slv8  := x"00";

begin

  GEN_CLKRAM : s7_cmt_sfs2               -- clock generator system ------------
    generic map (
      VCO_DIVIDE     => sys_conf_clkram_vcodivide,
      VCO_MULTIPLY   => sys_conf_clkram_vcomultiply,
      OUT_DIVIDE     => sys_conf_clkram_outdivide,
      OUT1_DIVIDE    => sys_conf_clksys_outdivide,
      CLKIN_PERIOD   => 10.0,
      CLKIN_JITTER   => 0.01,
      STARTUP_WAIT   => false,
      GEN_TYPE       => sys_conf_clkram_gentype)
    port map (
      CLKIN   => I_CLK100,
      CLKFX   => CLK200,
      CLKFX1  => CLK,
      LOCKED  => open
    );
  
  
  CLKDIV_CLK : clkdivce                 -- usec/msec clock divider system ----
    generic map (
      CDUWIDTH => 7,                    -- good for up to 127 MHz !
      USECDIV  => sys_conf_clksys_mhz,
      MSECDIV  => 1000)
    port map (
      CLK     => CLK,
      CE_USEC => CE_USEC,
      CE_MSEC => CE_MSEC
    );

  GEN_CLKSER : s7_cmt_sfs               -- clock generator serport------------
    generic map (
      VCO_DIVIDE     => sys_conf_clkser_vcodivide,
      VCO_MULTIPLY   => sys_conf_clkser_vcomultiply,
      OUT_DIVIDE     => sys_conf_clkser_outdivide,
      CLKIN_PERIOD   => 10.0,
      CLKIN_JITTER   => 0.01,
      STARTUP_WAIT   => false,
      GEN_TYPE       => sys_conf_clkser_gentype)
    port map (
      CLKIN   => I_CLK100,
      CLKFX   => CLKS,
      LOCKED  => open
    );

  CLKDIV_CLKS : clkdivce                -- usec/msec clock divider serport ---
    generic map (
      CDUWIDTH => 7,
      USECDIV  => sys_conf_clkser_mhz,
      MSECDIV  => 1000)
    port map (
      CLK     => CLKS,
      CE_USEC => open,
      CE_MSEC => CES_MSEC
    );

  IOB_RS232 : bp_rs232_4line_iob
    port map (
      CLK     => CLKS,
      RXD     => RXD,
      TXD     => TXD,
      CTS_N   => CTS_N,
      RTS_N   => RTS_N,
      I_RXD   => I_RXD,
      O_TXD   => O_TXD,
      I_CTS_N => I_CTS_N,
      O_RTS_N => O_RTS_N
    );

  HIO : sn_humanio
    generic map (
      SWIDTH   => 16,
      BWIDTH   =>  5,
      LWIDTH   => 16,
      DCWIDTH  =>  3)
    port map (
      CLK     => CLK,
      RESET   => '0',
      CE_MSEC => CE_MSEC,
      SWI     => SWI,
      BTN     => BTN,
      LED     => LED,
      DSP_DAT => DSP_DAT,
      DSP_DP  => DSP_DP,
      I_SWI   => I_SWI,
      I_BTN   => I_BTN,
      O_LED   => O_LED,
      O_ANO_N => O_ANO_N,
      O_SEG_N => O_SEG_N
    );

  RLINK : rlink_sp2c
    generic map (
      BTOWIDTH     => 6,                --  64 cycles access timeout
      RTAWIDTH     => 12,
      SYSID        => sysid_proj & sysid_board & sysid_vers,
      IFAWIDTH     => 5,                --  32 word input fifo
      OFAWIDTH     => 5,                --  32 word output fifo
      ENAPIN_RLMON => sbcntl_sbf_rlmon,
      ENAPIN_RBMON => sbcntl_sbf_rbmon,
      CDWIDTH      => 12,
      CDINIT       => sys_conf_ser2rri_cdinit,
      RBMON_AWIDTH => 0,
      RBMON_RBADDR => x"ffe8")
    port map (
      CLK      => CLK,
      CE_USEC  => CE_USEC,
      CE_MSEC  => CE_MSEC,
      CE_INT   => CE_MSEC,
      RESET    => GBL_RESET,
      CLKS     => CLKS,
      CES_MSEC => CES_MSEC,
      ENAXON   => SWI(1),
      ESCFILL  => '0',
      RXSD     => RXD,
      TXSD     => TXD,
      CTS_N    => CTS_N,
      RTS_N    => RTS_N,
      RB_MREQ  => RB_MREQ,
      RB_SRES  => RB_SRES,
      RB_LAM   => RB_LAM,
      RB_STAT  => RB_STAT,
      RL_MONI  => open,
      SER_MONI => SER_MONI
    );

  TST : entity work.tst_sram
    generic map (
      RB_ADDR => slv(to_unsigned(2#0000000000000000#,16)),
      AWIDTH  => 22)
    port map (
      CLK       => CLK,
      RESET     => GBL_RESET,
      RB_MREQ   => RB_MREQ,
      RB_SRES   => RB_SRES_TST,
      RB_STAT   => RB_STAT,
      RB_LAM    => RB_LAM_TST,
      SWI       => SWI(7 downto 0),
      BTN       => BTN(3 downto 0),
      LED       => LED(7 downto 0),
      DSP_DAT   => DSP_DAT(15 downto 0),
      MEM_RESET => MEM_RESET,
      MEM_REQ   => MEM_REQ,
      MEM_WE    => MEM_WE,
      MEM_BUSY  => MEM_BUSY,
      MEM_ACK_R => MEM_ACK_R,
      MEM_ACK_W => MEM_ACK_W,
      MEM_ACT_R => MEM_ACT_R,
      MEM_ACT_W => MEM_ACT_W,
      MEM_ADDR  => MEM_ADDR,
      MEM_BE    => MEM_BE,
      MEM_DI    => MEM_DI,
      MEM_DO    => MEM_DO
    );

  FAKECRAMCTL : nx_cram_memctl_as
    generic map (
      IOBATTR      => "false",
      CEFIRSTCYCLE => '0',
      READ0DELAY   => sys_conf_memctl_read0delay,
      READ1DELAY   => sys_conf_memctl_read1delay,
      WRITEDELAY   => sys_conf_memctl_writedelay) 
    port map (
      CLK     => CLK,
      RESET   => MEM_RESET,
      REQ     => MEM_REQ,
      WE      => MEM_WE,
      BUSY    => MEM_BUSY,
      ACK_R   => MEM_ACK_R,
      ACK_W   => MEM_ACK_W,
      ACT_R   => MEM_ACT_R,
      ACT_W   => MEM_ACT_W,
      ADDR    => MEM_ADDR,
      BE      => MEM_BE,
      DI      => MEM_DI,
      DO      => MEM_DO,
      O_MEM_CE_N  => CRAM_CE_N,
      O_MEM_BE_N  => CRAM_BE_N,
      O_MEM_WE_N  => CRAM_WE_N,
      O_MEM_OE_N  => CRAM_OE_N,
      O_MEM_ADV_N => CRAM_ADV_N,
      O_MEM_CLK   => CRAM_CLK,
      O_MEM_CRE   => CRAM_CRE,            -- control register enable
                                          -- Ignored!  Means at init time a spurious
                      			              -- memory write will occur.
      I_MEM_WAIT  => '0',                 -- Neither supported by ram2ddr nor used
      O_MEM_ADDR  => CRAM_ADDR(22 downto 0),
      IO_MEM_DATA => CRAM_DATAC
    );

  RAM2DDR: ram2ddrxadc
    port map (
      CLK_200MHZ_I  => CLK200,
      RST_I         => GBL_RESET,
      DEVICE_TEMP_I => (others=>'0'),

      RAM_A    => CRAM_ADDR27,

      RAM_DQ_I => CRAM_DATAO,
      RAM_DQ_O => CRAM_DATAI,

      RAM_CEN  => CRAM_CE_N,
      RAM_OEN  => CRAM_OE_N,
      RAM_WEN  => CRAM_WE_N,
      RAM_UB   => CRAM_BE_N(1),
      RAM_LB   => CRAM_BE_N(0),

      -- DDR2 interface
      DDR2_ADDR  => O_DDR2_ADDR,
      DDR2_BA    => O_DDR2_BA,
      DDR2_RAS_N => O_DDR2_RAS_N,
      DDR2_CAS_N => O_DDR2_CAS_N,
      DDR2_WE_N  => O_DDR2_WE_N,
      DDR2_CK_P  => O_DDR2_CK_P,
      DDR2_CK_N  => O_DDR2_CK_N,
      DDR2_CKE   => O_DDR2_CKE,
      DDR2_CS_N  => O_DDR2_CS_N,
      DDR2_DM    => O_DDR2_DM,
      DDR2_ODT   => O_DDR2_ODT,
      DDR2_DQ    => IO_DDR2_DQ,
      DDR2_DQS_P => IO_DDR2_DQS_P,
      DDR2_DQS_N => IO_DDR2_DQS_N
    );

  UARB : rbd_usracc
    port map (
      CLK     => CLK,
      RB_MREQ => RB_MREQ,
      RB_SRES => RB_SRES_USRACC
    );

  RB_SRES_OR : rb_sres_or_2             -- rbus or ---------------------------
    port map (
      RB_SRES_1  => RB_SRES_TST,
      RB_SRES_2  => RB_SRES_USRACC,
      RB_SRES_OR => RB_SRES
    );
  
  RB_LAM(0) <= RB_LAM_TST;

  DSP_DP(3) <= not SER_MONI.txok;
  DSP_DP(2) <= SER_MONI.txact;
  DSP_DP(1) <= not SER_MONI.rxok;
  DSP_DP(0) <= SER_MONI.rxact;

  proc_cramdata: process (CRAM_OE_N, CRAM_DATAC, CRAM_DATAI)
  begin
    if CRAM_OE_N = '0' then
      CRAM_DATAC <= CRAM_DATAI;
      CRAM_DATAO <= (others => '0');
    else
      CRAM_DATAC <= (others => 'Z');
      CRAM_DATAO <= CRAM_DATAC;
    end if;
  end process proc_cramdata;

  DSP_DP(7 downto 4) <= "0010";
  DSP_DAT(31 downto 16) <= SER_MONI.abclkdiv(11 downto 0) &
                           '0' & SER_MONI.abclkdiv_f;

  CRAM_ADDR27 <= "000" & CRAM_ADDR & "0";

  -- setup unused outputs in nexys4
  O_RGBLED0 <= (others=>'0');
  O_RGBLED1 <= (others=>not I_BTNRST_N);
end syn;

