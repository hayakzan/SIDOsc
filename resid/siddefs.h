//  ---------------------------------------------------------------------------
//  This file is part of reSID, a MOS6581 SID emulator engine.
//  Copyright (C) 1998 - 2022  Dag Lem <resid@nimrod.no>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//  ---------------------------------------------------------------------------

#ifndef RESID_SIDDEFS_H
#define RESID_SIDDEFS_H

// Compilation configuration.
#define RESID_INLINING 1
#define RESID_INLINE inline
#define RESID_BRANCH_HINTS 1
#define RESID_FPGA_CODE 0

// Compiler specifics.
#define RESID_CONSTEVAL 
#define RESID_CONSTEXPR const
#define RESID_CONSTINIT 
#define HAVE_BUILTIN_EXPECT 1

// Added due to the VERSION issue.
#ifndef VERSION
#define VERSION "1.0"
#endif


// Branch prediction macros, lifted off the Linux kernel.
#if RESID_BRANCH_HINTS && HAVE_BUILTIN_EXPECT
#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)
#else
#define likely(x)      (x)
#define unlikely(x)    (x)
#endif

namespace reSID {

// We could have used the smallest possible data type for each SID register,
// however this would give a slower engine because of data type conversions.
// An int is assumed to be at least 32 bits (necessary in the types reg24
// and cycle_count). GNU does not support 16-bit machines
// (GNU Coding Standards: Portability between CPUs), so this should be
// a valid assumption.

typedef unsigned int reg4;
typedef unsigned int reg8;
typedef unsigned int reg12;
typedef unsigned int reg16;
typedef unsigned int reg24;

typedef int cycle_count;
typedef short short_point[2];
typedef double double_point[2];

enum chip_model { MOS6581, MOS8580 };

enum sampling_method { SAMPLE_FAST, SAMPLE_INTERPOLATE,
		       SAMPLE_RESAMPLE, SAMPLE_RESAMPLE_FASTMEM };

} // namespace reSID

extern "C"
{
#ifndef RESID_VERSION_CC
extern const char* resid_version_string;
#else
const char* resid_version_string = VERSION;
#endif
}

#endif // not RESID_SIDDEFS_H
