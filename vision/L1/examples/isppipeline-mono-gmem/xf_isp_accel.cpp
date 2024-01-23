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
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP> img_inp(height, width);
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_BLC> img_blc(height, width);
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_MBF> img_mbf(height, width);
    xf::cv::Mat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_GCM> img_gcm(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_QAD> img_qad(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_CLH> img_clh(height, width);
    xf::cv::Mat<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT> img_out(height, width);

// clang-format off
#pragma HLS DATAFLOW
// clang-format on

	CLAHE_T obj;
	const int Q_VAL = 1 << (XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC));
	float inputMax = (1 << (XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC))) - 1; // 65535.0f;
	float mul_fact = (inputMax / (inputMax - BLACK_LEVEL));

    xf::cv::Array2xfMat<INPUT_PTR_WIDTH, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(pntr_in, img_inp);
    xf::cv::blackLevelCorrection<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_BLC, 16, 15, 1>(img_inp, img_blc, BLACK_LEVEL, mul_fact);
    xf::cv::medianBlur<WINDOW_SIZE, XF_BORDER_REPLICATE, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_BLC, XF_CV_DEPTH_MBF>(img_blc, img_mbf);
    xf::cv::gaincontrol_mono<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_MBF, XF_CV_DEPTH_GCM>(img_mbf, img_gcm, lgain);
    xf::cv::xf_QuatizationDithering<XF_DST_T, XF_LTM_T, XF_HEIGHT, XF_WIDTH, 256, Q_VAL, XF_NPPC, XF_CV_DEPTH_GCM, XF_CV_DEPTH_QAD>(img_gcm, img_qad);
    obj.process(img_clh, img_qad, _lutw, _lutr, _clipCounter, height, width, clip, tilesY, tilesX);
    xf::cv::gammacorrection<XF_LTM_T, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_CLH, XF_CV_DEPTH_OUT>(img_clh, img_out, gamma_lut);
    xf::cv::xfMat2Array<OUTPUT_PTR_WIDTH, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_out, pntr_out);
}

/*********************************************************************************
 * Function:    ISPPipeline_accel
 * Parameters: 
 * Return:
 * Description:
 **********************************************************************************/
void ISPPipeline_accel(ap_uint<INPUT_PTR_WIDTH>* img_inp,
                       ap_uint<OUTPUT_PTR_WIDTH>* img_out,
                       uint16_t width,
                       uint16_t height,
                       uint16_t lgain,
                       uint16_t clip,
                       uint16_t tilesY,
                       uint16_t tilesX,
                       unsigned char gamma_lut[256]) {
// clang-format off
#pragma HLS INTERFACE m_axi     port=img_inp  offset=slave bundle=gmem1
#pragma HLS INTERFACE m_axi     port=img_out  offset=slave bundle=gmem2

#pragma HLS INTERFACE s_axilite port=width      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=height     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=lgain      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=clip       bundle=CTRL
#pragma HLS INTERFACE s_axilite port=tilesY     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=tilesX     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=gamma_lut  bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return     bundle=CTRL

#pragma HLS ARRAY_PARTITION variable=_lut1 dim=3 complete
#pragma HLS ARRAY_PARTITION variable=_lut2 dim=3 complete

// clang-format on
    if (!flag) {
        ISPpipeline(s_axis_video, m_axis_video, height, width, lgain, gamma_lut, _lut1, _lut2, _clipCounter, clip,
                    tilesX, tilesY);
        flag = 1;

    } else {
        ISPpipeline(s_axis_video, m_axis_video, height, width, lgain, gamma_lut, _lut2, _lut1, _clipCounter, clip,
                    tilesX, tilesY);
        flag = 0;
    }
}
