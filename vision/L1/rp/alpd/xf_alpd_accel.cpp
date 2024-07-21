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

#include "xf_alpd_types.h"


template <int TYPE, int ROWS, int COLS, int NPPC, int XFCVDEPTH_OUT>
void mat2axi(xf::cv::Mat<TYPE, ROWS, COLS, NPPC, XFCVDEPTH_OUT>& gray_mat, OutVideoStrm_t& gray_strm) {
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

template <int TYPE, int ROWS, int COLS, int NPPC, int XFCVDEPTH_BAYER>
void axi2mat(InVideoStrm_t& bayer_strm, xf::cv::Mat<TYPE, ROWS, COLS, NPPC, XFCVDEPTH_BAYER>& bayer_mat) {
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

template <int TYPE, int TYPE_M, int ROWS, int COLS, int NPPC, int XFCVDEPTH_BAYER>
void alpd_t(InVideoStrm_t& strm_in,
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

template <int TYPE_SRC,
          int TYPE_LTM,
          int TYPE_DST,
          int ROWS,
          int COLS,
          int NPPC_SRC,
          int NPPC_LTM,
          int NPPC_DST,
          int XFCVDEPTH_SRC,
          int XFCVDEPTH_LTM,
          int XFCVDEPTH_DST>
void alpd(xf::cv::Mat<TYPE_SRC, ROWS, COLS, NPPC_SRC, XFCVDEPTH_SRC>& mat_src,
          xf::cv::Mat<TYPE_LTM, ROWS, COLS, NPPC_LTM, XFCVDEPTH_LTM>& mat_dat,
          xf::cv::Mat<TYPE_DST, ROWS, COLS, NPPC_DST, XFCVDEPTH_DST>& mat_dst,
          ap_uint<1> do_job) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    int rows = mat_src.rows;
    int cols = mat_src.cols >> XF_BITSHIFT(NPPC_SRC);
    int src_depth = XF_DTPIXELDEPTH(TYPE_SRC, NPPC_SRC);
    int dat_depth = XF_DTPIXELDEPTH(TYPE_LTM, NPPC_LTM);
    int src_idx = 0, dat_idx = 0, dst_idx = 0;

    XF_TNAME(TYPE_SRC, NPPC_SRC) px_src;
    XF_TNAME(TYPE_LTM, NPPC_LTM) px_dat;
    XF_TNAME(TYPE_DST, NPPC_DST) px_dst;

NA_loop_row:
    for (int row = 0; row < rows; row++) {
    // clang-format off
#pragma HLS LOOP_TRIPCOUNT min=ROWS max=ROWS
#pragma HLS LOOP_FLATTEN off
    // clang-format on
    NA_loop_col:
        for (int col = 0; col < cols; col++) {
        // clang-format off
#pragma HLS LOOP_TRIPCOUNT min=COLS/NPPC_SRC max=COLS/NPPC_SRC
#pragma HLS pipeline
        // clang-format on

            px_dat = 0x0000;
#if 1 // Generally only even bit can be equal to 1 - from application notes
            px_dst = mat_src.read(src_idx++);
            if (do_job && px_dst.range(14, 13)) {
                px_dst.range(14, 13) = 0;
                px_dat.range(7, 0) = 0xFF;
            }
#elif
            px_src = mat_src.read(src_idx++);
            for (int npc = 0; npc < NPPC_SRC; npc++) {
                for (int rs = 0; rs < 1; rs++) {
                    int src_str = (rs + npc) * src_depth;
                    int dat_str = (rs + npc) * dat_depth;
                    px_dst.range(src_str + (src_depth - 1), src_str) = px_src.range(src_str + (src_depth - 1), src_str);
                    if (px_dst.range(src_str + 14, src_str + 13)) {
                        px_dst.range(src_str + 14, src_str + 13) = 0;
                        px_dat.range(dat_str + (dat_depth - 1), dat_str) = 0xFF;
                    }
                }
            }
#endif

            mat_dat.write(dat_idx++, px_dat);
            mat_dst.write(dst_idx++, px_dst);
        }
    }
    return;
}

/*********************************************************************************
 * Function:    ALPD_accel
 * Parameters:  Stream of input/output pixels, image resolution
 * Return:
 * Description:
 **********************************************************************************/
void ALPD_accel(InVideoStrm_t& s_axis_video,
                OutVideoStrm_t& m_axis_video,
                ap_uint<OUTPUT_PTR_WIDTH>* out_pntr,
                uint16_t width,
                uint16_t height,
                uint8_t ctrl) {
    // clang-format off
#pragma HLS INTERFACE axis port=&s_axis_video register
#pragma HLS INTERFACE axis port=&m_axis_video register
#pragma HLS INTERFACE m_axi port=out_pntr offset=slave bundle=gmem

#pragma HLS INTERFACE s_axilite port=width      
#pragma HLS INTERFACE s_axilite port=height     
#pragma HLS INTERFACE s_axilite port=ctrl       
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> img_src(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_LTM> img_dat(height, width);
    xf::cv::Mat<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT> img_dst(height, width);

    // clang-format off
#pragma HLS DATAFLOW
    // clang-format on

    ap_uint<8> mode = (ap_uint<8>)ctrl;
    ap_uint<1> do_job = mode.range(0, 0); // Do JOB, otherwise pass to output

    axi2mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(s_axis_video, img_src);
    alpd<XF_SRC_T, XF_LTM_T, XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_NPPC, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_LTM,
         XF_CV_DEPTH_OUT>(img_src, img_dat, img_dst, do_job);
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_LTM>(img_dat, out_pntr);
    mat2axi<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_dst, m_axis_video);
}
