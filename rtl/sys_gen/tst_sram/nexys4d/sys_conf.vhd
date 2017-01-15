-- $Id: sys_conf.vhd 788 2016-07-16 22:23:23Z mueller $
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
-- Package Name:   sys_conf
-- Description:    Definitions for sys_tst_sram_n4 (for synthesis)
--
-- Dependencies:   -
-- Tool versions:  viv 2014.4-2016.4; ghdl 0.29-0.33
-- Revision History: 
-- Date         Rev Version  Comment
-- XXX NOCOMMIT
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

use work.slvtypes.all;
use work.nxcramlib.all;

package sys_conf is

  constant sys_conf_clkram_vcodivide   : positive :=   1;
  constant sys_conf_clkram_vcomultiply : positive :=  12;   -- vco 1200 MHz
  constant sys_conf_clkram_outdivide   : positive :=   6;   -- ram  200 MHz
  constant sys_conf_clkram_gentype     : string   := "PLL";

  constant sys_conf_clksys_outdivide   : positive :=  12;   -- sys  100 MHz

  -- dual clock design, clkser = 120 MHz
  constant sys_conf_clkser_vcodivide   : positive :=   1;
  constant sys_conf_clkser_vcomultiply : positive :=  12;   -- vco 1200 MHz
  constant sys_conf_clkser_outdivide   : positive :=  10;   -- sys  120 MHz
  constant sys_conf_clkser_gentype     : string   := "PLL";
  
  constant sys_conf_ser2rri_defbaud : integer := 115200;   -- default 115k baud

  -- derived constants
  
  constant sys_conf_clkram : integer :=
    ((100000000/sys_conf_clkram_vcodivide)*sys_conf_clkram_vcomultiply) /
    sys_conf_clkram_outdivide;
  constant sys_conf_clkram_mhz : integer := sys_conf_clkram/1000000;

  constant sys_conf_clksys : integer :=
    ((100000000/sys_conf_clkram_vcodivide)*sys_conf_clkram_vcomultiply) /
    sys_conf_clksys_outdivide;
  constant sys_conf_clksys_mhz : integer := sys_conf_clksys/1000000;

  constant sys_conf_clkser : integer :=
     ((100000000/sys_conf_clkser_vcodivide)*sys_conf_clkser_vcomultiply) /
    sys_conf_clkser_outdivide;
  constant sys_conf_clkser_mhz : integer := sys_conf_clkser/1000000;

  constant sys_conf_ser2rri_cdinit : integer :=
    (sys_conf_clkser/sys_conf_ser2rri_defbaud)-1;

  constant sys_conf_memctl_read0delay : positive :=
              ram2ddr_read0delay(sys_conf_clksys_mhz);
  constant sys_conf_memctl_read1delay : positive := 
              ram2ddr_read1delay(sys_conf_clksys_mhz);
  constant sys_conf_memctl_writedelay : positive := 
              ram2ddr_writedelay(sys_conf_clksys_mhz);

end package sys_conf;
