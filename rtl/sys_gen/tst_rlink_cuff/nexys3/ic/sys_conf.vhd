-- $Id: sys_conf.vhd 538 2013-10-06 17:21:25Z mueller $
--
-- Copyright 2013- by Walter F.J. Mueller <W.F.J.Mueller@gsi.de>
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
-- Description:    Definitions for sys_tst_rlink_cuff_ic_n3 (for synthesis)
--
-- Dependencies:   -
-- Tool versions:  xst 13.3, 14.6; ghdl 0.29
-- Revision History: 
-- Date         Rev Version  Comment
-- 2013-10-06   538   1.1    pll support, use clksys_vcodivide ect
-- 2013-01-04   469   1.0    Initial version 
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

use work.slvtypes.all;

package sys_conf is

  constant sys_conf_clksys_vcodivide   : positive :=   1;
  constant sys_conf_clksys_vcomultiply : positive :=   1;   -- dcm  100 MHz
  constant sys_conf_clksys_outdivide   : positive :=   1;   -- sys  100 MHz
  constant sys_conf_clksys_gentype     : string   := "DCM";

  constant sys_conf_ser2rri_defbaud : integer := 115200;   -- default 115k baud
  constant sys_conf_hio_debounce : boolean := true;    -- instantiate debouncers

  constant sys_conf_fx2_type : string := "ic2";

  -- dummy values defs for generic parameters of as controller
  constant sys_conf_fx2_rdpwldelay : positive := 1;
  constant sys_conf_fx2_rdpwhdelay : positive := 1;
  constant sys_conf_fx2_wrpwldelay : positive := 1;
  constant sys_conf_fx2_wrpwhdelay : positive := 1;
  constant sys_conf_fx2_flagdelay  : positive := 1;

  -- pktend timer setting
  --   petowidth=10 -> 2^10 30 MHz clocks -> ~33 usec (normal operation)
  constant sys_conf_fx2_petowidth  : positive := 10;

  constant sys_conf_fx2_ccwidth  : positive := 5;
   
  -- derived constants
  
  constant sys_conf_clksys : integer :=
    ((100000000/sys_conf_clksys_vcodivide)*sys_conf_clksys_vcomultiply) /
    sys_conf_clksys_outdivide;
  constant sys_conf_clksys_mhz : integer := sys_conf_clksys/1000000;

  constant sys_conf_ser2rri_cdinit : integer :=
    (sys_conf_clksys/sys_conf_ser2rri_defbaud)-1;

end package sys_conf;
