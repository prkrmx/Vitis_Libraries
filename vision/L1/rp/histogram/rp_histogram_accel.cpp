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


/************************************************************************************
 * Function:    AXIVideo2BayerMat
 * Parameters:  Multiple bayerWindow.getval AXI Stream, User Stream, Image Resolution
 * Return:      None
 * Description: Read data from multiple pixel/clk AXI stream into user defined stream
 ************************************************************************************/
template <int TYPE, int ROWS, int COLS, int NPPC, int XFCVDEPTH_BAYER>
void AXIVideo2BayerMat(InVideoStrm_t& bayer_strm, xf::cv::Mat<TYPE, ROWS, COLS, NPPC, XFCVDEPTH_BAYER>& bayer_mat) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    InVideoStrmBus_t axi;

    const int m_pix_width = XF_PIXELWIDTH(TYPE, NPPC) * XF_NPIXPERCYCLE(NPPC);

    int rows = bayer_mat.rows;
    int cols = bayer_mat.cols >> XF_BITSHIFT(NPPC);
    int idx = 0;

    bool start = false;
    bool last = false;

loop_start_hunt:
    while (!start) {
        // clang-format off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=0 max=0
        // clang-format on

        bayer_strm >> axi;
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
                bayer_strm >> axi;
            }

            last = axi.last.to_bool();

            bayer_mat.write(idx++, axi.data(m_pix_width - 1, 0));
        }

    loop_last_hunt:
        while (!last) {
            // clang-format off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=0 max=0
            // clang-format on

            bayer_strm >> axi;
            last = axi.last.to_bool();
        }
    }

    return;
}

template <int TYPE, int ROWS, int COLS, int NPPC, int XFCVDEPTH_OUT>
void GrayMat2AXIvideo(xf::cv::Mat<TYPE, ROWS, COLS, NPPC, XFCVDEPTH_OUT>& gray_mat, OutVideoStrm_t& gray_strm) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    OutVideoStrmBus_t axi;

    int rows = gray_mat.rows;
    int cols = gray_mat.cols >> XF_BITSHIFT(NPPC);
    int idx = 0;

    XF_TNAME(TYPE, NPPC) srcpixel;

    const int m_pix_width = XF_PIXELWIDTH(TYPE, NPPC) * XF_NPIXPERCYCLE(NPPC);

    int depth = XF_DTPIXELDEPTH(TYPE, NPPC);

    bool sof = true; // Indicates start of frame

loop_row_mat2axi:
    for (int i = 0; i < rows; i++) {
        // clang-format off
#pragma HLS loop_tripcount avg=ROWS max=ROWS
    // clang-format on
    loop_col_mat2axi:
        for (int j = 0; j < cols; j++) {
            // clang-format off
#pragma HLS loop_flatten off
#pragma HLS pipeline II = 1
#pragma HLS loop_tripcount avg=COLS/NPPC max=COLS/NPPC
            // clang-format on
            if (sof) {
                axi.user = 1;
            } else {
                axi.user = 0;
            }

            if (j == cols - 1) {
                axi.last = 1;
            } else {
                axi.last = 0;
            }

            axi.data = 0;

            srcpixel = gray_mat.read(idx++);

            for (int npc = 0; npc < NPPC; npc++) {
                for (int rs = 0; rs < 1; rs++) {
                    int start = (rs + npc) * depth;

                    axi.data(start + (depth - 1), start) = srcpixel.range(start + (depth - 1), start);
                }
            }

            axi.keep = -1;
            gray_strm << axi;

            sof = false;
        }
    }

    return;
}

template <int TYPE, int TYPE_M, int ROWS, int COLS, int NPPC, int XFCVDEPTH_BAYER>
void ALPD_doJob(InVideoStrm_t& strm_in,
                OutVideoStrm_t& strm_out,
                xf::cv::Mat<TYPE_M, ROWS, COLS, NPPC, XFCVDEPTH_BAYER>& mat_dat,
                ap_uint<1> do_job,
                uint16_t threshold) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    InVideoStrmBus_t axi;

    const int m_pix_width = XF_PIXELWIDTH(TYPE, NPPC) * XF_NPIXPERCYCLE(NPPC);

    int rows = mat_dat.rows;
    int cols = mat_dat.cols >> XF_BITSHIFT(NPPC);
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

            if (do_job) {
                uint16_t ss = 0;
                // if (axi.data(15, 0) > threshold) {
                if (axi.data(14, 13)) {
                    ss = 0xFF;
                    axi.data(14, 13) = 0;
                }
                // if (axi.data(31, 16) > threshold) {
                if (axi.data(30, 29)) {
                    ss |= 0xFF00;
                    axi.data(30, 29) = 0;
                }
                mat_dat.write(idx++, ss);
            }
            strm_out << axi;
        }

    loop_last_hunt:
        while (!last) {
            // clang-format off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=0 max=0
            // clang-format on

            strm_in >> axi;
            strm_out << axi;
            last = axi.last.to_bool();
        }
    }

    return;
}

/*********************************************************************************
 * Function:    Histogram_accel
 * Parameters:  Stream of input/output pixels, image resolution
 * Return:
 * Description:
 **********************************************************************************/
void Histogram_accel(InVideoStrm_t& s_axis_video,
                OutVideoStrm_t& m_axis_video,
                ap_uint<OUTPUT_PTR_WIDTH>* out_pntr,
                uint16_t width,
                uint16_t height,
                uint16_t threshold,
                uint8_t ctrl) {
    // clang-format off
#pragma HLS INTERFACE axis port=&s_axis_video register
#pragma HLS INTERFACE axis port=&m_axis_video register
#pragma HLS INTERFACE m_axi port=out_pntr offset=slave bundle=gmem

#pragma HLS INTERFACE s_axilite port=width      
#pragma HLS INTERFACE s_axilite port=height     
#pragma HLS INTERFACE s_axilite port=threshold  
#pragma HLS INTERFACE s_axilite port=ctrl       
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_LTM> img_dat(height, width);
    // clang-format off
#pragma HLS DATAFLOW
    // clang-format on

    ap_uint<8> mode = (ap_uint<8>)ctrl;
    ap_uint<1> do_job = mode.range(0, 0); // Do JOB, otherwise pass to output

    ALPD_doJob<XF_SRC_T, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_LTM>(s_axis_video, m_axis_video, img_dat, do_job, threshold);
    if (do_job)
        xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_LTM>(img_dat, out_pntr);
}
