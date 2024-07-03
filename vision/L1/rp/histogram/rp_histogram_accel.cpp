/*
 */

#include "rp_histogram_types.h"

template <int ROWS, int COLS, int NPPC>
void CalculateHistogram(InVideoStrm_t& strm_in,
                        uint32_t* histogram,
                        uint16_t width,
                        uint16_t height,
                        uint16_t* min,
                        uint16_t* max,
                        uint64_t* sum) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    InVideoStrmBus_t axi;

    int rows = height;
    int cols = width >> XF_BITSHIFT(NPPC);
    int idx = 0;

    bool start = false;
    bool last = false;

    uint16_t _min = 8192, _max = 0;
    uint64_t _sum = 0;

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
            if (axi.data(15, 0) < _min) _min = axi.data(15, 0);
            if (axi.data(31, 16) < _min) _min = axi.data(31, 16);
            if (axi.data(15, 0) > _max) _max = axi.data(15, 0);
            if (axi.data(31, 16) > _max) _max = axi.data(31, 16);
            _sum += axi.data(15, 0);
            _sum += axi.data(31, 16);
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
    *min = _min;
    *max = _max;
    *sum = _sum;
    return;
}
static constexpr int __XF_DEPTH_PTR = (8192 * (XF_CHANNELS(XF_SRC_T, XF_NPPC)));

/*********************************************************************************
 * Function:    Histogram_accel
 * Parameters:  Stream of input pixels, resolution
 * Return:
 * Description:
 **********************************************************************************/
void Histogram_accel(InVideoStrm_t& s_axis_video,
                     unsigned int* histogram,
                     uint16_t width,
                     uint16_t height,
                     uint16_t* min,
                     uint16_t* max,
                     uint64_t* sum) {
    // clang-format off
#pragma HLS INTERFACE axis  port=&s_axis_video register
#pragma HLS INTERFACE m_axi port=histogram offset=slave bundle=gmem depth=__XF_DEPTH_PTR

#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=min
#pragma HLS INTERFACE s_axilite port=max
#pragma HLS INTERFACE s_axilite port=sum
#pragma HLS INTERFACE s_axilite port=return
    // clang-format on

    CalculateHistogram<XF_HEIGHT, XF_WIDTH, XF_NPPC>(s_axis_video, histogram, width, height, min, max, sum);
}
