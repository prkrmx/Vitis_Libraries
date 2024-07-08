/*
 * Copyright 2019 Xilinx, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _RP_NUC_TYPES_H_
#define _RP_NUC_TYPES_H_

// --------------------------------------------------------------------
// Required files
// --------------------------------------------------------------------
#include "hls_stream.h"
#include "ap_int.h"
#include "common/xf_common.hpp"
#include "ap_axi_sdata.h"
#include "common/xf_axi_io.hpp"
#include "rp_config_params.h"

// Requried Vision modules
#include "imgproc/xf_cvt_color.hpp"


// --------------------------------------------------------------------
// Macros definitions
// --------------------------------------------------------------------

// Useful macro functions definitions
#define _DATA_WIDTH_(_T, _N) (XF_PIXELWIDTH(_T, _N) * XF_NPIXPERCYCLE(_N))
#define _BYTE_ALIGN_(_N) ((((_N) + 7) / 8) * 8)

#define IN_DATA_WIDTH _DATA_WIDTH_(XF_SRC_T, XF_NPPC)
#define OUT_DATA_WIDTH _DATA_WIDTH_(XF_DST_T, XF_NPPC)

#define AXI_WIDTH_IN _BYTE_ALIGN_(IN_DATA_WIDTH)
#define AXI_WIDTH_OUT _BYTE_ALIGN_(OUT_DATA_WIDTH)

#define BLACK_LEVEL 32
#define MAX_PIX_VAL (1 << (XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC))) - 1

// --------------------------------------------------------------------
// Internal types
// --------------------------------------------------------------------
// Input/Output AXI video buses
typedef ap_axiu<AXI_WIDTH_IN, 1, 1, 1> VideoStrmBus_t;

// Input/Output AXI video stream
typedef hls::stream<VideoStrmBus_t> VideoStrm_t;

// --------------------------------------------------------------------
// Prototype
// Applying NUC data to input stream pixels 
// --------------------------------------------------------------------
// top level function for HW synthesis
void NUC_accel(VideoStrm_t& s_axis_video,
               VideoStrm_t& m_axis_video,
               ap_uint<INPUT_PTR_WIDTH>* gain_pntr,
               ap_uint<INPUT_PTR_WIDTH>* offset_pntr,
               uint16_t width,
               uint16_t height,
               uint8_t ctrl);
#endif //_RP_NUC_TYPES_H_
