/*
 */

#include "rp_nuc_types.h"

template <int TYPE_SRC,
          int TYPE_TAB,
          int TYPE_DST,
          int ROWS,
          int COLS,
          int NPPC_SRC,
          int NPPC_TAB,
          int NPPC_DST,
          int XFCVDEPTH_SRC,
          int XFCVDEPTH_TAB,
          int XFCVDEPTH_DST>
void nuc(xf::cv::Mat<TYPE_SRC, ROWS, COLS, NPPC_SRC, XFCVDEPTH_SRC>& mat_src,
               xf::cv::Mat<TYPE_TAB, ROWS, COLS, NPPC_TAB, XFCVDEPTH_TAB>& mat_gain,
               xf::cv::Mat<TYPE_TAB, ROWS, COLS, NPPC_TAB, XFCVDEPTH_TAB>& mat_ofst,
               xf::cv::Mat<TYPE_DST, ROWS, COLS, NPPC_DST, XFCVDEPTH_DST>& mat_dst) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    int rows = mat_src.rows;
    int cols = mat_src.cols >> XF_BITSHIFT(NPPC_SRC);
    int depth = XF_DTPIXELDEPTH(TYPE_SRC, NPPC_SRC);
    int src_idx = 0, gain_idx = 0, ofst_idx = 0, dst_idx = 0;

    XF_TNAME(TYPE_SRC, NPPC_SRC) px_src;
    XF_TNAME(TYPE_TAB, NPPC_TAB) px_gain;
    XF_TNAME(TYPE_TAB, NPPC_TAB) px_ofst;
    XF_TNAME(TYPE_DST, NPPC_DST) px_dst;

NA_loop_row:
    for (int row = 0; row < rows; row++) {
        // clang-format off
#pragma HLS loop_tripcount avg=ROWS max=ROWS
    // clang-format on
    NA_loop_col:
        for (int col = 0; col < cols; col++) {
            // clang-format off
#pragma HLS loop_flatten off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=COLS/NPPC_SRC max=COLS/NPPC_SRC
            // clang-format on

            px_src = mat_src.read(src_idx++);
            for (int npc = 0; npc < NPPC_SRC; npc++) {
                for (int rs = 0; rs < 1; rs++) {
                    px_gain = mat_gain.read(gain_idx++);
                    px_ofst = mat_ofst.read(ofst_idx++);
                    int start = (rs + npc) * depth;

                    uint16_t val = px_src.range(start + (depth - 1), start);
                    px_dst.range(start + (depth - 1), start) = (uint16_t)(((val * px_gain) >> 10) + px_ofst);
                }
            }
            mat_dst.write(dst_idx++, px_dst);
        }
    }
    return;
}

template <int SRC_T, int DST_T, int ROWS, int COLS, int NPPC, int XFCVDEPTH_IN, int XFCVDEPTH_OUT>
void fifo(xf::cv::Mat<SRC_T, ROWS, COLS, NPPC, XFCVDEPTH_IN>& mat_src,
               xf::cv::Mat<DST_T, ROWS, COLS, NPPC, XFCVDEPTH_OUT>& mat_dst) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    int rows = mat_src.rows;
    int cols = mat_src.cols >> XF_BITSHIFT(NPPC);

    int readindex = 0;
    int writeindex = 0;

FC_loop_row:
    for (int row = 0; row < rows; row++) {
        // clang-format off
#pragma HLS LOOP_TRIPCOUNT min=ROWS max=ROWS
#pragma HLS LOOP_FLATTEN off
    // clang-format on
    FC_loop_col:
        for (int col = 0; col < cols; col++) {
            // clang-format off
#pragma HLS LOOP_TRIPCOUNT min=COLS/NPPC max=COLS/NPPC
#pragma HLS pipeline
            // clang-format on
            XF_TNAME(SRC_T, NPPC) tmp_src;
            tmp_src = mat_src.read(readindex++);
            mat_dst.write(writeindex++, tmp_src);
        }
    }
}

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

/*********************************************************************************
 * Function:    NUC_accel
 * Parameters:  Stream of input pixels, resolution, control
 * Return:
 * Description:
 **********************************************************************************/
void NUC_accel(InVideoStrm_t& s_axis_video,
               OutVideoStrm_t& m_axis_video,
               ap_uint<INPUT_PTR_WIDTH>* gain_pntr,
               ap_uint<INPUT_PTR_WIDTH>* offset_pntr,
               uint16_t width,
               uint16_t height,
               uint8_t ctrl) {
    // clang-format off
#pragma HLS INTERFACE axis port=&s_axis_video register
#pragma HLS INTERFACE axis port=&m_axis_video register
#pragma HLS INTERFACE m_axi port=gain_pntr offset=slave bundle=gmem_gain
#pragma HLS INTERFACE m_axi port=offset_pntr offset=slave bundle=gmem_offset

#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=ctrl
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> img_src(height, width);
    xf::cv::Mat<XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC_T, XF_CV_DEPTH_TAB> img_gain(height, width);
    xf::cv::Mat<XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC_T, XF_CV_DEPTH_TAB> img_offset(height, width);
    xf::cv::Mat<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT> img_dst(height, width);

    // clang-format off
#pragma HLS DATAFLOW
    // clang-format on

    axi2mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(s_axis_video, img_src);
    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC_T, XF_CV_DEPTH_TAB>(gain_pntr,
                                                                                                    img_gain);
    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC_T, XF_CV_DEPTH_TAB>(offset_pntr,
                                                                                                    img_offset);

    ap_uint<8> mode = (ap_uint<8>)ctrl;
    ap_uint<1> do_job = mode.range(0, 0); // Do JOB, otherwise pass to output

    if (do_job)
        nuc<XF_SRC_T, XF_TAB_T, XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_NPPC_T, XF_NPPC, XF_CV_DEPTH_INP,
            XF_CV_DEPTH_TAB, XF_CV_DEPTH_OUT>(img_src, img_gain, img_offset, img_dst);
    else
        fifo<XF_SRC_T, XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_OUT>(img_src, img_dst);

    mat2axi<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_dst, m_axis_video);
}