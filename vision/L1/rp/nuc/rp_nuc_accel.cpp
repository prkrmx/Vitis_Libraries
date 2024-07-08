/*
 */

#include "rp_nuc_types.h"

template <int TYPE, int TYPE_I, int ROWS, int COLS, int NPPC, int NPPC_I, int XFCVDEPTH_BAYER>
void nuc_apply(VideoStrm_t& strm_in,
               VideoStrm_t& strm_out,
               xf::cv::Mat<TYPE_I, ROWS, COLS, NPPC_I, XFCVDEPTH_BAYER>& mat_gain,
               xf::cv::Mat<TYPE_I, ROWS, COLS, NPPC_I, XFCVDEPTH_BAYER>& mat_offset,
               ap_uint<1> do_job) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    VideoStrmBus_t axi;

    int rows = mat_gain.rows;
    int cols = mat_gain.cols >> XF_BITSHIFT(NPPC);
    int idx = 0;
    int depth = XF_DTPIXELDEPTH(TYPE, NPPC);

    XF_TNAME(TYPE_I, NPPC_I) gain;
    XF_TNAME(TYPE_I, NPPC_I) offset;

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
                for (int npc = 0; npc < NPPC; npc++) {
                    gain = mat_gain.read(idx + npc);
                    offset = mat_offset.read(idx + npc);
                    int _start = npc * depth;
                    int _stop = _start + (depth - 1);
                    int _gain = gain.range(31, 0);
                    int _offset = offset.range(31, 0);
                    uint16_t val = axi.data(_stop, _start);
                    uint16_t dd = (val * _gain) >> 10;
                    uint16_t dds = dd + _offset;
                    axi.data(_stop, _start) = dds;
                }

                // uint16_t _gain = gain.range(15,0);
                // int16_t _offset = (int16_t)offset.range(15,0);
                // uint16_t val = axi.data(15, 0);
                // val = ((val * _gain) >> 10)  + _offset;
                // axi.data(15, 0) = val;

                //  _gain = gain.range(31, 16);
                //  _offset = (int16_t)offset.range(31, 16);
                // val = axi.data(31, 16);
                // val = ((val * _gain) >> 10)  + _offset;
                // axi.data(31, 16) = val;
            }
            idx+=2;
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
 * Function:    NUC_accel
 * Parameters:  Stream of input pixels, resolution, control
 * Return:
 * Description:
 **********************************************************************************/
void NUC_accel(VideoStrm_t& s_axis_video,
               VideoStrm_t& m_axis_video,
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
    xf::cv::Mat<XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC1, XF_CV_DEPTH_INP> img_gain(height, width);
    xf::cv::Mat<XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC1, XF_CV_DEPTH_INP> img_offset(height, width);

    // clang-format off
#pragma HLS DATAFLOW
    // clang-format on

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC1, XF_CV_DEPTH_INP>(offset_pntr,
                                                                                                  img_offset);
    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC1, XF_CV_DEPTH_INP>(gain_pntr, img_gain);

    ap_uint<8> mode = (ap_uint<8>)ctrl;
    ap_uint<1> do_job = mode.range(0, 0); // Do JOB, otherwise pass to output

    nuc_apply<XF_SRC_T, XF_TAB_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_NPPC1, XF_CV_DEPTH_INP>(s_axis_video, m_axis_video, img_gain, img_offset, do_job);
}
