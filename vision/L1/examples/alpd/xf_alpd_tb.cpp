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

#include "common/xf_headers.hpp"
#include "xf_alpd_types.h"

using namespace std;

/*********************************************************************************
 * Function:    GrayMat2AXIvideo
 * Description: 16 bit Mat to stream
 **********************************************************************************/
static void GrayMat2AXIvideo(cv::Mat& img, InVideoStrm_t& AXI_video_strm) {
    int i, j, k, l;

    unsigned short cv_pix;
    ap_axiu<AXI_WIDTH_IN, 1, 1, 1> axi;
    int depth = XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC);

    for (i = 0; i < img.rows; i++) {
        for (j = 0; j < img.cols; j += XF_NPPC) {
            if ((i == 0) && (j == 0)) {
                axi.user = 1;
            } else {
                axi.user = 0;
            }
            if (j == (img.cols - XF_NPPC)) {
                axi.last = 1;
            } else {
                axi.last = 0;
            }
            axi.data = -1;
            for (l = 0; l < XF_NPPC; l++) {
                cv_pix = img.at<unsigned short>(i, j + l);
                xf::cv::AXISetBitFields(axi, (l)*depth, depth, (unsigned short)cv_pix);
            }
            axi.keep = -1;
            AXI_video_strm << axi;
        }
    }
}

/*********************************************************************************
 * Function:    GrayAXIvideo2Mat
 * Description: Extract pixels from stream and write to open 16 bit gray CV Image
 **********************************************************************************/
static void GrayAXIvideo2Mat(OutVideoStrm_t& AXI_video_strm, cv::Mat& img) {
    int i, j, k, l;
    ap_axiu<AXI_WIDTH_OUT, 1, 1, 1> axi;
    unsigned short cv_pix;
    int depth = XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC);
    bool sof = 0;

    for (i = 0; i < img.rows; i++) {
        for (j = 0; j < img.cols / XF_NPPC; j++) { // 4 pixels read per iteration
            AXI_video_strm >> axi;
            if ((i == 0) && (j == 0)) {
                if (axi.user.to_int() == 1) {
                    sof = 1;
                } else {
                    j--;
                }
            }
            if (sof) {
                for (l = 0; l < XF_NPPC; l++) {
                    cv_pix = axi.data(l * depth + depth - 1, l * depth);
                    img.at<unsigned short>(i, (XF_NPPC * j + l)) = cv_pix;
                }
            } // if(sof)
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Invalid Number of Arguments!\nUsage:\n");
        fprintf(stderr, "<Executable Name> <input image path> \n");
        return -1;
    }

    cv::Mat img_in, img_out;

    InVideoStrm_t src_axi;
    OutVideoStrm_t dst_axi;

    // read input image
    img_in = cv::imread(argv[1], -1);

    if (img_in.data == NULL) {
        fprintf(stderr, "Cannot open image at %s\n", argv[1]);
        return 0;
    }

    std::cout << "Input image height : " << img_in.rows << std::endl;
    std::cout << "Input image width  : " << img_in.cols << std::endl;
    std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC) << std::endl;
    std::cout << "Input Image Channels:" << XF_CHANNELS(XF_SRC_T, XF_NPPC) << std::endl;
    std::cout << "NPPC:" << XF_NPPC << std::endl;
    std::cout << "IN DEPTH:" << img_in.depth() << std::endl;

    img_out.create(img_in.rows, img_in.cols, CV_16UC1);
    imwrite("input.png", img_in);

    unsigned char ctrl = 0;
    unsigned int gamma_lut[256];
    unsigned short threshold = 8192;

    GrayMat2AXIvideo(img_in, src_axi);

    // Call IP Processing function
    ALPD_accel(src_axi, dst_axi, img_in.cols, img_in.rows, ctrl, threshold, gamma_lut);

    // Convert processed image back to CV image, then to XVID image
    GrayAXIvideo2Mat(dst_axi, img_out);

    std::cout << "OUT DEPTH:" << img_out.depth() << std::endl;
    imwrite("output.png", img_out);

    return 0;
}
