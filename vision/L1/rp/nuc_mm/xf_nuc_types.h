/*
 * File:     xf_nuc_types.h
 * Notes:    
 *
 * Author:   Engr. Max Parker
 * Created:  Thu Jul 11 2024
 *
 * Copyright (C) 2024 RP Optical Lab
 */


#ifndef _RP_NUC_MM_TYPES_H_
#define _RP_NUC_MM_TYPES_H_

// --------------------------------------------------------------------
// Required files
// --------------------------------------------------------------------
// #include "hls_stream.h"
// #include "ap_int.h"
// #include "common/xf_common.hpp"
// #include "ap_axi_sdata.h"

#include "common/xf_utility.hpp"
#include "xf_nuc_config.h"

// --------------------------------------------------------------------
// Macros definitions
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// Prototype
// Applying 2P NUC
// --------------------------------------------------------------------
// top level function for HW synthesis
void NUC2P_Accel(ap_uint<INPUT_PTR_WIDTH>*  src_pntr,
                 ap_uint<IN_TBG_PTR_WIDTH>* gain_pntr,
                 ap_uint<IN_TBO_PTR_WIDTH>* offset_pntr,
                 ap_uint<OUTPUT_PTR_WIDTH>* dst_pntr,
                 uint16_t width,
                 uint16_t height,
                 uint8_t  ctrl);
#endif //_RP_NUC_MM_TYPES_H_