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

#include "xf_isp_types.h"

static bool flag = 0;

#define CLAHE_T                                                                                           \
    xf::cv::clahe::CLAHEImpl<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, CLIPLIMIT, TILES_Y_MAX, TILES_X_MAX, \
                             XF_CV_DEPTH_QAD, XF_CV_DEPTH_CLH, TILES_Y_MIN, TILES_X_MIN>

static constexpr int HIST_COUNTER_BITS = CLAHE_T::HIST_COUNTER_BITS;
static constexpr int CLIP_COUNTER_BITS = CLAHE_T::CLIP_COUNTER_BITS;

static ap_uint<HIST_COUNTER_BITS> _lut1[TILES_Y_MAX][TILES_X_MAX][(XF_NPIXPERCYCLE(XF_NPPC) << 1)]
                                       [1 << XF_DTPIXELDEPTH(XF_LTM_T, XF_NPPC)];
static ap_uint<HIST_COUNTER_BITS> _lut2[TILES_Y_MAX][TILES_X_MAX][(XF_NPIXPERCYCLE(XF_NPPC) << 1)]
                                       [1 << XF_DTPIXELDEPTH(XF_LTM_T, XF_NPPC)];
static ap_uint<CLIP_COUNTER_BITS> _clipCounter[TILES_Y_MAX][TILES_X_MAX];

/*********************************************************************************
 * Function:    Copy
 * Parameters:  
 * Return:
 * Description: Copy source image to destination
 **********************************************************************************/
template <int SRC_T, int DST_T, int ROWS, int COLS, int NPPC, int XFCVDEPTH_SRC, int XFCVDEPTH_DST>
void copy(xf::cv::Mat<SRC_T, ROWS, COLS, NPPC, XFCVDEPTH_SRC>& mat_src,
          xf::cv::Mat<DST_T, ROWS, COLS, NPPC, XFCVDEPTH_DST>& mat_dst) {
    // clang-format off
#pragma HLS INLINE OFF
    // clang-format on

    int rows = mat_src.rows;
    int cols = mat_src.cols >> XF_BITSHIFT(NPPC);

    int i_src = 0, i_dst = 0;

fifo_loop_row:
    for (int row = 0; row < rows; row++) {
        // clang-format off
#pragma HLS LOOP_TRIPCOUNT min=ROWS max=ROWS
#pragma HLS LOOP_FLATTEN off
    // clang-format on
    fifo_loop_col:
        for (int col = 0; col < cols; col++) {
            // clang-format off
#pragma HLS LOOP_TRIPCOUNT min=COLS/NPPC max=COLS/NPPC
#pragma HLS pipeline
            // clang-format on
            XF_TNAME(SRC_T, NPPC) tmp_src;
            tmp_src = mat_src.read(i_src++);
            mat_dst.write(i_dst++, tmp_src);
        }
    }
    return;
}

template <int SRC_T, int DST_T, int ROWS, int COLS, int NPC = 1, int XFCVDEPTH_IN, int XFCVDEPTH_OUT>
void fifo_copy(xf::cv::Mat<SRC_T, ROWS, COLS, NPC, XFCVDEPTH_IN>& demosaic_out,
               xf::cv::Mat<DST_T, ROWS, COLS, NPC, XFCVDEPTH_OUT>& ltm_in,
               unsigned short height,
               unsigned short width) {
// clang-format off
#pragma HLS INLINE OFF
    // clang-format on
    ap_uint<13> row, col;
    int readindex = 0, writeindex = 0;

    ap_uint<13> img_width = width >> XF_BITSHIFT(NPC);

Row_Loop:
    for (row = 0; row < height; row++) {
// clang-format off
#pragma HLS LOOP_TRIPCOUNT min=ROWS max=ROWS
#pragma HLS LOOP_FLATTEN off
    // clang-format on
    Col_Loop:
        for (col = 0; col < img_width; col++) {
// clang-format off
#pragma HLS LOOP_TRIPCOUNT min=COLS/NPC max=COLS/NPC
#pragma HLS pipeline
            // clang-format on
            XF_TNAME(SRC_T, NPC) tmp_src;
            tmp_src = demosaic_out.read(readindex++);
            ltm_in.write(writeindex++, tmp_src);
        }
    }
}

/************************************************************************************
 * Function:    ISPpipeline
 * Parameters:  
 * Return:      None
 * Description: 
 ************************************************************************************/
void ISPpipeline(ap_uint<INPUT_PTR_WIDTH>* pntr_in,
                 ap_uint<OUTPUT_PTR_WIDTH>* pntr_out,
                 unsigned short height,
                 unsigned short width,
                 uint16_t lgain,
                 unsigned char gamma_lut[256],
                 ap_uint<HIST_COUNTER_BITS> _lutw[TILES_Y_MAX][TILES_X_MAX][(XF_NPIXPERCYCLE(XF_NPPC) << 1)]
                                                 [1 << XF_DTPIXELDEPTH(XF_LTM_T, XF_NPPC)],
                 ap_uint<HIST_COUNTER_BITS> _lutr[TILES_Y_MAX][TILES_X_MAX][(XF_NPIXPERCYCLE(XF_NPPC) << 1)]
                                                 [1 << XF_DTPIXELDEPTH(XF_LTM_T, XF_NPPC)],
                 ap_uint<CLIP_COUNTER_BITS> _clipCounter[TILES_Y_MAX][TILES_X_MAX],
                 uint16_t clip,
                 uint16_t tilesY,
                 uint16_t tilesX) {
// clang-format off
#pragma HLS INLINE OFF
// clang-format on
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> img_src(height, width);
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_BLC> img_blc(height, width);
    // xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_MBF> img_mbf(height, width);
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_GCM> img_gcm(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_QAD> img_qad(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_CLH> img_clh(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT> img_dst(height, width);

// clang-format off
#pragma HLS DATAFLOW
// clang-format on

	CLAHE_T obj;
	const int Q_VAL = 1 << (XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC));
	float inputMax = (1 << (XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC))) - 1; // 65535.0f;
	float mul_fact = (inputMax / (inputMax - BLACK_LEVEL));

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(pntr_in, img_src);
    xf::cv::blackLevelCorrection<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_BLC, 16, 15, 1>(img_src, img_blc, BLACK_LEVEL, mul_fact);
    // xf::cv::medianBlur<WINDOW_SIZE, XF_BORDER_REPLICATE, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_BLC, XF_CV_DEPTH_MBF>(img_blc, img_mbf);
    xf::cv::gaincontrol_mono<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_MBF, XF_CV_DEPTH_GCM>(img_blc, img_gcm, lgain);
    xf::cv::xf_QuatizationDithering<XF_SRC_T, XF_LTM_T, XF_HEIGHT, XF_WIDTH, 256, Q_VAL, XF_NPPC, XF_CV_DEPTH_GCM, XF_CV_DEPTH_QAD>(img_gcm, img_qad);
    obj.process(img_clh, img_qad, _lutw, _lutr, _clipCounter, height, width, clip, tilesY, tilesX);
    xf::cv::gammacorrection<XF_LTM_T, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_CLH, XF_CV_DEPTH_OUT>(img_clh, img_dst, gamma_lut);
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_dst, pntr_out);
}

/*********************************************************************************
 * Function:    ISPPipeline_accel
 * Parameters: 
 * Return:
 * Description:
 **********************************************************************************/
void ISPPipeline_accel(ap_uint<INPUT_PTR_WIDTH>* in_pntr,
                       ap_uint<OUTPUT_PTR_WIDTH>* out_pntr,
                       uint16_t width,
                       uint16_t height,
                       uint16_t lgain,
                       uint16_t clip,
                       uint16_t tilesY,
                       uint16_t tilesX,
                       unsigned char gamma_lut[256]) {
// clang-format off
#pragma HLS INTERFACE m_axi port=in_pntr  offset=slave bundle=gmem1
#pragma HLS INTERFACE m_axi port=out_pntr offset=slave bundle=gmem2

#pragma HLS INTERFACE s_axilite port=width
#pragma HLS INTERFACE s_axilite port=height
#pragma HLS INTERFACE s_axilite port=lgain
#pragma HLS INTERFACE s_axilite port=clip
#pragma HLS INTERFACE s_axilite port=tilesY
#pragma HLS INTERFACE s_axilite port=tilesX
#pragma HLS INTERFACE s_axilite port=gamma_lut
#pragma HLS INTERFACE s_axilite port=return

#pragma HLS ARRAY_PARTITION variable=_lut1 dim=3 complete
#pragma HLS ARRAY_PARTITION variable=_lut2 dim=3 complete

// clang-format on
    if (!flag) {
        ISPpipeline(in_pntr, out_pntr, height, width, lgain, gamma_lut, _lut1, _lut2, _clipCounter, clip, tilesX, tilesY);
        flag = 1;

    } else {
        ISPpipeline(in_pntr, out_pntr, height, width, lgain, gamma_lut, _lut2, _lut1, _clipCounter, clip, tilesX, tilesY);
        flag = 0;
    }
}
