/*
 */

#include "rp_histogram_types.h"


template <int TYPE, int ROWS, int COLS, int NPPC, int XFCVDEPTH_BAYER>
void CalculateHistogram(xf::cv::Mat<TYPE, ROWS, COLS, NPPC, XFCVDEPTH_BAYER>& mat_src,
                        uint32_t* histogram,
                        uint16_t* min,
                        uint16_t* max,
                        uint64_t* sum) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    int rows = mat_src.rows;
    int cols = mat_src.cols >> XF_BITSHIFT(NPPC);
    int idx = 0;

    uint16_t _min = 8192, _max = 0;
    uint64_t _sum = 0;

loop_row_hist:
    for (int i = 0; i < rows; i++) {
        // clang-format off
#pragma HLS loop_tripcount avg=ROWS max=ROWS
        // clang-format on
    loop_col_hist:
        for (int j = 0; j < cols; j++) {
            // clang-format off
#pragma HLS loop_flatten off
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount avg=COLS/NPPC max=COLS/NPPC
            // clang-format on

            XF_TNAME(TYPE, NPPC) tmp_src;
            tmp_src = mat_src.read(idx++);

            histogram[tmp_src]++;
            if (tmp_src < _min) _min = tmp_src;
            if (tmp_src > _max) _max = tmp_src;
            _sum += tmp_src;
        }
    }
    *min = _min;
    *max = _max;
    *sum = _sum;
    return;
}
static constexpr int __XF_DEPTH_PTR = (8192 * 4);

/*********************************************************************************
 * Function:    Histogram_accel
 * Parameters:  Stream of input pixels, resolution
 * Return:
 * Description:
 **********************************************************************************/
void Histogram_accel(ap_uint<INPUT_PTR_WIDTH>* src_pntr,
                     unsigned int* histogram,
                     uint16_t width,
                     uint16_t height,
                     uint16_t* min,
                     uint16_t* max,
                     uint64_t* sum) {
    // clang-format off
#pragma HLS INTERFACE m_axi port=src_pntr  offset=slave bundle=gmem_src
#pragma HLS INTERFACE m_axi port=histogram offset=slave bundle=gmem_hist depth=__XF_DEPTH_PTR

#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=min
#pragma HLS INTERFACE s_axilite port=max
#pragma HLS INTERFACE s_axilite port=sum
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> src_img(height, width);

    // clang-format off
#pragma HLS DATAFLOW
    // clang-format on

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(src_pntr, src_img);

    CalculateHistogram<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(src_img, histogram, min, max, sum);
}
