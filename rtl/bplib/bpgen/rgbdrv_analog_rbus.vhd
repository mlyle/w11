-- $Id: rgbdrv_analog_rbus.vhd 734 2016-02-20 22:43:20Z mueller $
--
-- Copyright 2016- by Walter F.J. Mueller <W.F.J.Mueller@gsi.de>
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
-- Module Name:   rgbdrv_analog_rbus - syn
-- Description:   rgb analog from rbus
--
-- Dependencies:   bpgen/rgbdrv_analog
--
-- Test bench:     -
--
-- Target Devices: generic
-- Tool versions:  ise 14.7; viv 2015.4; ghdl 0.31
--
-- Revision History: 
-- Date         Rev Version  Comment
-- 2016-02-20   724   1.0    Initial version
------------------------------------------------------------------------------
--
-- rbus registers:
--
-- Addr   Bits  Name        r/w/f  Function
--   00         red         r/w/-  red channel
--   01         green       r/w/-  green channel
--   10         blue        r/w/-  blue channel
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.slvtypes.all;
use work.rblib.all;
use work.bpgenlib.all;

-- ----------------------------------------------------------------------------

entity rgbdrv_analog_rbus is   -- rgb analog from rbus
  generic (
    DWIDTH : positive := 8;             -- dimmer width
    RB_ADDR : slv16 := slv(to_unsigned(16#0000#,16)));
  port (
    CLK : in slbit;                     -- clock
    RESET : in slbit := '0';            -- reset
    RB_MREQ : in rb_mreq_type;          -- rbus: request
    RB_SRES : out rb_sres_type;         -- rbus: response
    RGBCNTL : in slv3;                  -- rgb control
    DIMCNTL : in slv(DWIDTH-1 downto 0);-- dim control
    O_RGBLED : out slv3                 -- pad-o: rgb led
  );
end rgbdrv_analog_rbus;

architecture syn of rgbdrv_analog_rbus is
  
  type regs_type is record
    rbsel : slbit;                      -- rbus select
    dimr  : slv(DWIDTH-1 downto 0);     -- dim red
    dimg  : slv(DWIDTH-1 downto 0);     -- dim green
    dimb  : slv(DWIDTH-1 downto 0);     -- dim blue
  end record regs_type;

  constant dimzero : slv(DWIDTH-1 downto 0) := (others=>'0');

  constant regs_init : regs_type := (
    '0',                                -- rbsel
    dimzero,                            -- dimr
    dimzero,                            -- dimg
    dimzero                             -- dimb
  );

  signal R_REGS : regs_type := regs_init;  -- state registers
  signal N_REGS : regs_type := regs_init;  -- next value state regs

  subtype  dim_rbf  is integer range DWIDTH-1 downto 0;

  constant rbaddr_dimr:   slv2 := "00";     --  0    r/w/-
  constant rbaddr_dimg:   slv2 := "01";     --  1    r/w/-
  constant rbaddr_dimb:   slv2 := "10";     --  2    r/w/-

begin

  assert DWIDTH<=16 
    report "assert (DWIDTH<=16)"
    severity failure;

  RGB : rgbdrv_analog
    generic map (
      DWIDTH   => DWIDTH)
    port map (
      CLK      => CLK,
      RESET    => RESET,
      RGBCNTL  => RGBCNTL,
      DIMCNTL  => DIMCNTL,
      DIMR     => R_REGS.dimr,
      DIMG     => R_REGS.dimg,
      DIMB     => R_REGS.dimb,
      O_RGBLED => O_RGBLED
    );
  
  proc_regs: process (CLK)
  begin

    if rising_edge(CLK) then
      if RESET = '1' then
        R_REGS <= regs_init;
      else
        R_REGS <= N_REGS;
      end if;
    end if;

  end process proc_regs;
  
  proc_next: process (R_REGS, RB_MREQ)

    variable r : regs_type := regs_init;
    variable n : regs_type := regs_init;

    variable irb_ack  : slbit := '0';
    variable irb_busy : slbit := '0';
    variable irb_err  : slbit := '0';
    variable irb_dout : slv16 := (others=>'0');
    variable irbena   : slbit := '0';
    
  begin

    r := R_REGS;
    n := R_REGS;

    irb_ack  := '0';
    irb_busy := '0';
    irb_err  := '0';
    irb_dout := (others=>'0');

    irbena  := RB_MREQ.re or RB_MREQ.we;

    -- rbus address decoder
    n.rbsel := '0';
    if RB_MREQ.aval='1' and RB_MREQ.addr(15 downto 2)=RB_ADDR(15 downto 2) then
      n.rbsel := '1';
    end if;

    -- rbus transactions
    if r.rbsel = '1' then
      irb_ack := irbena;                  -- ack all accesses

      case RB_MREQ.addr(1 downto 0) is
        
        when rbaddr_dimr =>
          irb_dout(dim_rbf) := r.dimr;
          if RB_MREQ.we = '1' then
            n.dimr := RB_MREQ.din(dim_rbf);
          end if;          

        when rbaddr_dimg =>
          irb_dout(dim_rbf) := r.dimg;
          if RB_MREQ.we = '1' then
            n.dimg := RB_MREQ.din(dim_rbf);
          end if;          

        when rbaddr_dimb =>
          irb_dout(dim_rbf) := r.dimb;
          if RB_MREQ.we = '1' then
            n.dimb := RB_MREQ.din(dim_rbf);
          end if;          

        when others =>
          irb_ack := '0';
          
      end case;

    end if;

    N_REGS       <= n;

    RB_SRES      <= rb_sres_init;
    RB_SRES.ack  <= irb_ack;
    RB_SRES.busy <= irb_busy;
    RB_SRES.err  <= irb_err;
    RB_SRES.dout <= irb_dout;

  end process proc_next;

end syn;
