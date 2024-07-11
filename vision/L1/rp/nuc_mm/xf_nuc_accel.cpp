/*
 * File:     xf_nuc_accel.cpp
 * Notes:    
 *
 * Author:   Engr. Max Parker
 * Created:  Thu Jul 11 2024
 *
 * Copyright (C) 2024 RP Optical Lab
 */


#include "xf_nuc_types.h"

template <int TYPE_SR,
          int TYPE_TG,
          int TYPE_TO,
          int TYPE_DS,
          int ROWS,
          int COLS,
          int NPPC,
          int XFCVDEPTH_SR,
          int XFCVDEPTH_TG,
          int XFCVDEPTH_TO,
          int XFCVDEPTH_DS>
void nuc(xf::cv::Mat<TYPE_SR, ROWS, COLS, NPPC, XFCVDEPTH_SR>& mat_src,
         xf::cv::Mat<TYPE_TG, ROWS, COLS, NPPC, XFCVDEPTH_TG>& mat_gain,
         xf::cv::Mat<TYPE_TO, ROWS, COLS, NPPC, XFCVDEPTH_TO>& mat_offst,
         xf::cv::Mat<TYPE_DS, ROWS, COLS, NPPC, XFCVDEPTH_DS>& mat_dst) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    int rows = mat_src.rows;
    int cols = mat_src.cols >> XF_BITSHIFT(NPPC);
    int i_src = 0, i_gain = 0, i_offst = 0, i_dst = 0;

    int depth_mn = XF_DTPIXELDEPTH(TYPE_SR, NPPC);
    int depth_tg = XF_DTPIXELDEPTH(TYPE_TG, NPPC);
    int depth_to = XF_DTPIXELDEPTH(TYPE_TO, NPPC);
    
    // Chunks of data  
    XF_TNAME(TYPE_SR, NPPC) ch_src;
    XF_TNAME(TYPE_TG, NPPC) ch_gain;
    XF_TNAME(TYPE_TO, NPPC) ch_offst;
    XF_TNAME(TYPE_DS, NPPC) ch_dst;

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
            ch_gain = mat_gain.read(i_gain++);
            ch_offst = mat_offst.read(i_offst++);
            for (int npc = 0; npc < NPPC; npc++) {
                for (int rs = 0; rs < 1; rs++) {
                    int strt_mn = (rs + npc) * depth_mn;
                    int strt_tg = (rs + npc) * depth_tg;
                    uint16_t px_src = ch_src.range(strt_mn + (depth_mn - 1), strt_mn);
                    uint16_t px_gain = ch_gain.range(strt_tg + (depth_tg - 1), strt_tg);
                    int16_t px_offst = ch_offst.range(strt_tg + (depth_to - 1), strt_tg);
                    uint16_t px_dst = (uint16_t)(((px_src * px_gain) >> 10) + px_offst);
                    ch_dst.range(strt_mn + (depth_mn - 1), strt_mn) = px_dst;
                }
            }
            mat_dst.write(i_dst++, ch_dst);
        }
    }
    return;
}

/*********************************************************************************
 * Function:    NUC2P_Accel
 * Parameters:  
 * Return:
 * Description:
 **********************************************************************************/
void NUC2P_Accel(ap_uint<INPUT_PTR_WIDTH>* src_pntr,
                 ap_uint<IN_TBG_PTR_WIDTH>* gain_pntr,
                 ap_uint<IN_TBO_PTR_WIDTH>* offset_pntr,
                 ap_uint<OUTPUT_PTR_WIDTH>* dst_pntr,
                 uint16_t width,
                 uint16_t height) {
    // clang-format off
#pragma HLS INTERFACE m_axi port=src_pntr  offset=slave bundle=gmem_in
#pragma HLS INTERFACE m_axi port=gain_pntr offset=slave bundle=gmem_gain
#pragma HLS INTERFACE m_axi port=offset_pntr offset=slave bundle=gmem_offset
#pragma HLS INTERFACE m_axi port=dst_pntr offset=slave bundle=gmem_out

#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> img_src(height, width);
    xf::cv::Mat<XF_TBG_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_TBG> img_gain(height, width);
    xf::cv::Mat<XF_TBO_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_TBO> img_offset(height, width);
    xf::cv::Mat<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT> img_dst(height, width);

    // clang-format off
#pragma HLS DATAFLOW
    // clang-format on

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(src_pntr, img_src);
    xf::cv::Array2xfMat<IN_TBG_PTR_WIDTH, XF_TBG_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_TBG>(gain_pntr, img_gain);
    xf::cv::Array2xfMat<IN_TBO_PTR_WIDTH, XF_TBO_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_TBO>(offset_pntr, img_offset);

    nuc<XF_SRC_T, XF_TBG_T, XF_TBO_T, XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_TBG,
        XF_CV_DEPTH_TBO, XF_CV_DEPTH_OUT>(img_src, img_gain, img_offset, img_dst);

    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_dst, dst_pntr);
}
