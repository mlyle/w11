-- $Id: rgbdrv_analog.vhd 734 2016-02-20 22:43:20Z mueller $
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
-- Module Name:    rgbdrv_analog - syn
-- Description:    rgbled driver: analog channel
--
-- Dependencies:   -
-- Test bench:     -
-- Target Devices: generic
-- Tool versions:  viv 2015.4; ghdl 0.31
--
-- Revision History: 
-- Date         Rev Version  Comment
-- 2016-02-20   734   1.0    Initial version 
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.slvtypes.all;
use work.xlib.all;

entity rgbdrv_analog is                 -- rgbled driver: analog channel
  generic (
    DWIDTH : positive := 8);            -- dimmer width
  port (
    CLK : in slbit;                     -- clock
    RESET : in slbit := '0';            -- reset
    RGBCNTL : in slv3;                  -- rgb control
    DIMCNTL : in slv(DWIDTH-1 downto 0);-- dim control
    DIMR : in slv(DWIDTH-1 downto 0);   -- dim red
    DIMG : in slv(DWIDTH-1 downto 0);   -- dim green
    DIMB : in slv(DWIDTH-1 downto 0);   -- dim blue
    O_RGBLED : out slv3                 -- pad-o: rgb led
  );
end rgbdrv_analog;

architecture syn of rgbdrv_analog is

  signal R_RGB : slv3 := (others=>'0');  -- state registers
  signal N_RGB : slv3 := (others=>'0');  -- next value state regs

begin

  IOB_RGB : iob_reg_o_gen
    generic map (DWIDTH => 3)
    port map (CLK => CLK, CE => '1', DO => R_RGB, PAD => O_RGBLED);

  proc_regs: process (CLK)
  begin

    if rising_edge(CLK) then
      if RESET = '1' then
        R_RGB <= (others=>'0');
      else
        R_RGB <= N_RGB;
      end if;
    end if;

  end process proc_regs;


  proc_next: process (R_RGB, RGBCNTL, DIMCNTL, DIMR, DIMG, DIMB)
    variable irgb : slv3 := (others=>'0');
  begin

    irgb := (others=>'0');

    if unsigned(DIMCNTL) < unsigned(DIMR) then
      irgb(0) := RGBCNTL(0);
    end if;
    
    if unsigned(DIMCNTL) < unsigned(DIMG) then
      irgb(1) := RGBCNTL(1);
    end if;
    
    if unsigned(DIMCNTL) < unsigned(DIMB) then
      irgb(2) := RGBCNTL(2);
    end if;
    
    N_RGB <= irgb;
    
  end process proc_next;

  
end syn;
