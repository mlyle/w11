-- $Id: ibdr_sdreg.vhd 314 2010-07-09 17:38:41Z mueller $
--
-- Copyright 2007-2010 by Walter F.J. Mueller <W.F.J.Mueller@gsi.de>
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
-- Module Name:    ibdr_sdreg - syn
-- Description:    ibus dev(rem): Switch/Display register
--
-- Dependencies:   -
-- Test bench:     -
-- Target Devices: generic
-- Tool versions:  xst 8.1, 8.2, 9.1, 9.2, 10.1; ghdl 0.18-0.25
--
-- Synthesized (xst):
-- Date         Rev  ise         Target      flop lutl lutm slic t peri
-- 2009-07-11   232  10.1.03 K39 xc3s1000-4    32   39    0   29 s  2.5
--
-- Revision History: 
-- Date         Rev Version  Comment
-- 2010-06-11   303   1.1    use IB_MREQ.racc instead of RRI_REQ
-- 2008-08-22   161   1.0.4  use iblib
-- 2008-04-18   136   1.0.3  use RESET. Switch/Display not cleared by console
--                           reset or reset instruction, only by cpu_reset
-- 2008-01-20   112   1.0.2  use BRESET
-- 2008-01-05   110   1.0.1  rename IB_MREQ(ena->req) SRES(sel->ack, hold->busy)
--                           reorganize code, all in state_type/proc_next
-- 2007-12-31   108   1.0    Initial version 
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;

use work.slvtypes.all;
use work.iblib.all;

-- ----------------------------------------------------------------------------
entity ibdr_sdreg is                    -- ibus dev(rem): Switch/Display regs
                                        -- fixed address: 177570
  port (
    CLK : in slbit;                     -- clock
    RESET : in slbit;                   -- reset
    IB_MREQ : in ib_mreq_type;          -- ibus request
    IB_SRES : out ib_sres_type;         -- ibus response
    DISPREG : out slv16                 -- display register
  );
end ibdr_sdreg;

architecture syn of ibdr_sdreg is

  constant ibaddr_sdreg : slv16 := conv_std_logic_vector(8#177570#,16);

  type regs_type is record              -- state registers
    sreg : slv16;                       -- switch register
    dreg : slv16;                       -- display register
  end record regs_type;

  constant regs_init : regs_type := (
    (others=>'0'),
    (others=>'0')
  );

  signal R_REGS : regs_type := regs_init;
  signal N_REGS : regs_type := regs_init;

begin
  
  proc_regs: process (CLK)
  begin
    if CLK'event and CLK='1' then
      if RESET = '1' then
        R_REGS <= regs_init;
     else
        R_REGS <= N_REGS;
      end if;
    end if;
  end process proc_regs;

  proc_next : process (R_REGS, IB_MREQ)
    variable r : regs_type := regs_init;
    variable n : regs_type := regs_init;
    variable ibsel : slbit := '0';
    variable idout : slv16 := (others=>'0');
  begin

    r := R_REGS;
    n := R_REGS;

    ibsel := '0';
    idout := (others=>'0');
    
    -- ibus address decoder
    if IB_MREQ.req='1' and IB_MREQ.addr=ibaddr_sdreg(12 downto 1) then
      ibsel := '1';
    end if;

    -- ibus output driver
    if ibsel = '1' then
      if IB_MREQ.racc = '0' then
        idout := r.sreg;             -- cpu will read switch register
      else
        idout := r.dreg;             -- rri will read display register
      end if;
    end if;

    -- ibus write transactions
    if ibsel='1' and IB_MREQ.we='1' then
      if IB_MREQ.racc = '0' then     -- cpu will write display register
        if IB_MREQ.be1 = '1' then
          n.dreg(ibf_byte1) := IB_MREQ.din(ibf_byte1);
        end if;
        if IB_MREQ.be0 = '1' then
          n.dreg(ibf_byte0) := IB_MREQ.din(ibf_byte0);
        end if;
      else                          -- rri will write switch register
        n.sreg := IB_MREQ.din;        -- byte write not supported
      end if;
    end if;
    
    N_REGS <= n;

    IB_SRES.dout <= idout;
    IB_SRES.ack  <= ibsel;
    IB_SRES.busy <= '0';
    
    DISPREG <= r.dreg;

  end process proc_next;

    
end syn;
