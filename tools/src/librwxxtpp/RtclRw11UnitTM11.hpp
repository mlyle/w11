// $Id: RtclRw11UnitTM11.hpp 686 2015-06-04 21:08:08Z mueller $
//
// Copyright 2015- by Walter F.J. Mueller <W.F.J.Mueller@gsi.de>
//
// This program is free software; you may redistribute and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 2, or at your option any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY, without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for complete details.
// 
// Revision History: 
// Date         Rev Version  Comment
// 2015-05-17   683   1.0    Initial version
// ---------------------------------------------------------------------------


/*!
  \file
  \version $Id: RtclRw11UnitTM11.hpp 686 2015-06-04 21:08:08Z mueller $
  \brief   Declaration of class RtclRw11UnitTM11.
*/

#ifndef included_Retro_RtclRw11UnitTM11
#define included_Retro_RtclRw11UnitTM11 1

#include "librw11/Rw11UnitTM11.hpp"
#include "librw11/Rw11CntlTM11.hpp"

#include "RtclRw11UnitTape.hpp"
#include "RtclRw11UnitBase.hpp"

namespace Retro {

  class RtclRw11UnitTM11 : public RtclRw11UnitBase<Rw11UnitTM11>,
                           public RtclRw11UnitTape {
    public:
                    RtclRw11UnitTM11(Tcl_Interp* interp,
                                 const std::string& unitcmd,
                                 const boost::shared_ptr<Rw11UnitTM11>& spunit);
                   ~RtclRw11UnitTM11();

    protected:
  };
  
} // end namespace Retro

//#include "RtclRw11UnitTM11.ipp"

#endif
