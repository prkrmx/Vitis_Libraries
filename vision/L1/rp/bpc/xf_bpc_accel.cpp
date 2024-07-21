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

#include "xf_bpc_config.h"

//static constexpr int __XF_DEPTH =
//    (XF_HEIGHT * XF_WIDTH * (XF_PIXELWIDTH(XF_SRC_T, XF_NPPC)) / 8) / (INPUT_PTR_WIDTH / 8);

void bpc_accel(ap_uint<INPUT_PTR_WIDTH>*  in_pntr,
               ap_uint<OUTPUT_PTR_WIDTH>* out_pntr,
               uint16_t height,
               uint16_t width) {
// clang-format off
#pragma HLS INTERFACE m_axi port=in_pntr    offset=slave bundle=gmem1
#pragma HLS INTERFACE m_axi port=out_pntr   offset=slave bundle=gmem2

#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=return
// clang-format on

// clang-format off
#pragma HLS INLINE OFF
// clang-format on

    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> img_src(height, width);
    xf::cv::Mat<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT> img_dst(height, width);

// clang-format off
#pragma HLS DATAFLOW
// clang-format on

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(in_pntr, img_src);
    xf::cv::badpixelcorrection<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_OUT, 0, 0>(img_src, img_dst);
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_dst, out_pntr);
}
