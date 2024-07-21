/*
 * File:     xf_alpd_accel.cpp
 * Notes:
 *
 * Author:   Engr. Max Parker
 * Created:  Thu Jul 11 2024
 *
 * Copyright (C) 2024 RP Optical Lab
 */

#include "xf_alpd_types.h"

template <int TYPE_SRC,
          int TYPE_LTM,
          int TYPE_DST,
          int ROWS,
          int COLS,
          int NPPC,
          int XFCVDEPTH_SRC,
          int XFCVDEPTH_LTM,
          int XFCVDEPTH_DST>
void alpd(xf::cv::Mat<TYPE_SRC, ROWS, COLS, NPPC, XFCVDEPTH_SRC>& mat_src,
          xf::cv::Mat<TYPE_LTM, ROWS, COLS, NPPC, XFCVDEPTH_LTM>& mat_ltm,
          xf::cv::Mat<TYPE_DST, ROWS, COLS, NPPC, XFCVDEPTH_DST>& mat_dst) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    int rows = mat_src.rows;
    int cols = mat_src.cols >> XF_BITSHIFT(NPPC);
    int i_src = 0, i_ltm = 0, i_dst = 0;

    int depth_mn = XF_DTPIXELDEPTH(TYPE_SRC, NPPC);
    int depth_ltm = XF_DTPIXELDEPTH(TYPE_LTM, NPPC);

    // Chunks of data
    XF_TNAME(TYPE_SRC, NPPC) ch_src;
    XF_TNAME(TYPE_LTM, NPPC) ch_ltm;
    XF_TNAME(TYPE_DST, NPPC) ch_dst;

nuc_loop_row:
    for (int row = 0; row < rows; row++) {
        // clang-format off
#pragma HLS loop_tripcount avg=ROWS max=ROWS
        // clang-format on
    nuc_loop_col:
        for (int col = 0; col < cols; col++) {
            // clang-format off
#pragma HLS loop_flatten off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=COLS/NPPC max=COLS/NPPC
            // clang-format on

            ch_src = mat_src.read(i_src++);
            for (int npc = 0; npc < NPPC; npc++) {
                for (int rs = 0; rs < 1; rs++) {
                    int strt_mn = (rs + npc) * depth_mn;
                    int strt_ltm = (rs + npc) * depth_ltm;
                    ap_uint<16> data = ch_src.range(strt_mn + (depth_mn - 1), strt_mn);
                    ap_uint<8> ltm = 0;
                    if (data.range(14, 13)) {
                    	data.range(14, 13) = 0;
                    	ltm = 255;
                    }
                    ch_ltm.range(strt_ltm + (depth_ltm - 1), strt_ltm) = ltm;
                    ch_dst.range(strt_mn + (depth_mn - 1), strt_mn) = data;
                }
            }
            mat_ltm.write(i_ltm++, ch_ltm);
            mat_dst.write(i_dst++, ch_dst);
        }
    }
    return;
}

/*********************************************************************************
 * Function:    ALPD_Accel
 * Parameters:  
 * Return:
 * Description:
 **********************************************************************************/
void ALPD_Accel(ap_uint<INPUT_PTR_WIDTH>* src_pntr,
                ap_uint<OUTLTM_PTR_WIDTH>* ltm_pntr,
                ap_uint<OUTPUT_PTR_WIDTH>* dst_pntr,
                uint16_t width,
                uint16_t height) {
    // clang-format off
#pragma HLS INTERFACE m_axi port=src_pntr offset=slave bundle=gmem_in
#pragma HLS INTERFACE m_axi port=ltm_pntr offset=slave bundle=gmem_ltm
#pragma HLS INTERFACE m_axi port=dst_pntr offset=slave bundle=gmem_out

#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> img_src(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_LTM> img_ltm(height, width);
    xf::cv::Mat<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT> img_dst(height, width);

    // clang-format off
#pragma HLS DATAFLOW
    // clang-format on

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(src_pntr, img_src);

    alpd<XF_SRC_T, XF_LTM_T, XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_LTM, XF_CV_DEPTH_OUT>(
        img_src, img_ltm, img_dst);

    xf::cv::xfMat2Array<OUTLTM_PTR_WIDTH, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_LTM>(img_ltm, ltm_pntr);
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_dst, dst_pntr);
}
