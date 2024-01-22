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

// static uint32_t histogram0[1][256] = {0};
// static uint32_t histogram1[1][256] = {0};

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

void ISPpipeline(InVideoStrm_t& s_axis_video,
                 OutVideoStrm_t& m_axis_video,
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
                 uint16_t tilesX,
                 unsigned char mode_reg) {
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
	ap_uint<8> mode = (ap_uint<8> ) mode_reg;
	ap_uint<1> do_job = mode.range(0, 0); // Do JOB, otherwise pass to output

	AXIVideo2BayerMat<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP>(s_axis_video, img_inp);
	if (do_job) {
		xf::cv::blackLevelCorrection<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_INP, XF_CV_DEPTH_BLC, 16, 15, 1>(
			img_inp, img_blc, BLACK_LEVEL, mul_fact);
		xf::cv::medianBlur<WINDOW_SIZE, XF_BORDER_REPLICATE, XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_BLC, XF_CV_DEPTH_MBF>(
			img_blc, img_mbf);
		xf::cv::gaincontrol_mono<XF_SRC_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_MBF, XF_CV_DEPTH_GCM>(
			img_mbf, img_gcm, lgain);
		xf::cv::xf_QuatizationDithering<XF_DST_T, XF_LTM_T, XF_HEIGHT, XF_WIDTH, 256, Q_VAL, XF_NPPC, XF_CV_DEPTH_GCM, XF_CV_DEPTH_QAD>(
			img_gcm, img_qad);
		obj.process(img_clh, img_qad, _lutw, _lutr, _clipCounter, height, width, clip, tilesY, tilesX);
		xf::cv::gammacorrection<XF_LTM_T, XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_CLH, XF_CV_DEPTH_OUT>(
			img_clh, img_out, gamma_lut);
		GrayMat2AXIvideo<XF_LTM_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_out, m_axis_video);
    } else {
		GrayMat2AXIvideo<XF_DST_T, XF_HEIGHT, XF_WIDTH, XF_NPPC, XF_CV_DEPTH_OUT>(img_inp, m_axis_video);
	}
}

/*********************************************************************************
 * Function:    ISPPipeline_accel
 * Parameters:  Stream of input/output pixels, image resolution
 * Return:
 * Description:
 **********************************************************************************/
void ISPPipeline_accel(InVideoStrm_t& s_axis_video,
                       OutVideoStrm_t& m_axis_video,
                       uint16_t width,
                       uint16_t height,
                       uint16_t lgain,
                       uint16_t clip,
                       uint16_t tilesY,
                       uint16_t tilesX,
                       unsigned char mode_reg,
                       unsigned char gamma_lut[256]) {
    // Create AXI Streaming Interfaces for the core
    // clang-format off
#pragma HLS INTERFACE axis port=&s_axis_video register
#pragma HLS INTERFACE axis port=&m_axis_video register

#pragma HLS INTERFACE s_axilite port=width      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=height     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=lgain      bundle=CTRL
#pragma HLS INTERFACE s_axilite port=clip       bundle=CTRL
#pragma HLS INTERFACE s_axilite port=tilesY     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=tilesX     bundle=CTRL
#pragma HLS INTERFACE s_axilite port=mode_reg   bundle=CTRL
#pragma HLS INTERFACE s_axilite port=gamma_lut  bundle=CTRL

// #pragma HLS INTERFACE s_axilite port=lgain      bundle=CTRL offset=0x0020
// #pragma HLS INTERFACE s_axilite port=clip       bundle=CTRL offset=0x0024
// #pragma HLS INTERFACE s_axilite port=tilesY     bundle=CTRL offset=0x0028
// #pragma HLS INTERFACE s_axilite port=tilesX     bundle=CTRL offset=0x002C
// #pragma HLS INTERFACE s_axilite port=mode_reg   bundle=CTRL offset=0x0030
// #pragma HLS INTERFACE s_axilite port=gamma_lut  bundle=CTRL offset=0x0100

#pragma HLS INTERFACE s_axilite port=return bundle=CTRL
// clang-format on
// clang-format off
#pragma HLS ARRAY_PARTITION variable=_lut1 dim=3 complete
#pragma HLS ARRAY_PARTITION variable=_lut2 dim=3 complete

	// clang-format on
    if (!flag) {
        ISPpipeline(s_axis_video, m_axis_video, height, width, lgain, gamma_lut, _lut1, _lut2, _clipCounter, clip,
                    tilesX, tilesY, mode_reg);
        flag = 1;

    } else {
        ISPpipeline(s_axis_video, m_axis_video, height, width, lgain, gamma_lut, _lut2, _lut1, _clipCounter, clip,
                    tilesX, tilesY, mode_reg);
        flag = 0;
    }
}
