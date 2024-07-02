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

#include "rp_histogram_types.h"

template <int ROWS, int COLS, int NPPC>
void CalculateHistogram(InVideoStrm_t& strm_in, uint32_t* histogram) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    InVideoStrmBus_t axi;

    int rows = ROWS;
    int cols = COLS >> XF_BITSHIFT(NPPC);
    int idx = 0;

    bool start = false;
    bool last = false;

loop_start_hunt:
    while (!start) {
        // clang-format off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=0 max=0
        // clang-format on

        strm_in >> axi;
        start = axi.user.to_bool();
    }

loop_row_axi2mat:
    for (int i = 0; i < rows; i++) {
        last = false;
        // clang-format off
#pragma HLS loop_tripcount avg=ROWS max=ROWS
    // clang-format on
    loop_col_zxi2mat:
        for (int j = 0; j < cols; j++) {
            // clang-format off
#pragma HLS loop_flatten off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=COLS/NPPC max=COLS/NPPC
            // clang-format on

            if (start || last) {
                start = false;
            } else {
                strm_in >> axi;
            }

            last = axi.last.to_bool();
            histogram[axi.data(15, 0)]++;
            histogram[axi.data(31, 16)]++;
        }

    loop_last_hunt:
        while (!last) {
            // clang-format off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=0 max=0
            // clang-format on

            strm_in >> axi;
            last = axi.last.to_bool();
        }
    }

    return;
}
static constexpr int __XF_DEPTH_PTR = (8192 * (XF_CHANNELS(XF_SRC_T, XF_NPPC)));

/*********************************************************************************
 * Function:    Histogram_accel
 * Parameters:  Stream of input pixels, resolution
 * Return:
 * Description:
 **********************************************************************************/
void Histogram_accel(InVideoStrm_t& s_axis_video, unsigned int* histogram, uint16_t width, uint16_t height) {
    // clang-format off
#pragma HLS INTERFACE axis  port=&s_axis_video register
#pragma HLS INTERFACE m_axi port=histogram offset=slave bundle=gmem depth=__XF_DEPTH_PTR

#pragma HLS INTERFACE s_axilite port=width      
#pragma HLS INTERFACE s_axilite port=height     
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    CalculateHistogram<XF_HEIGHT, XF_WIDTH, XF_NPPC>(s_axis_video, histogram);
}
