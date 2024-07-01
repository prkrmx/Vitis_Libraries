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

#define THRESHOLD 8192
#define CTRL 0
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

void ALPD_Detector(cv::Mat& img, cv::Mat& img_ret, unsigned short threshold) {
    unsigned int MASK = 0x2000;
    for (int row = 0; row < img.rows; row++) {
        for (int col = 0; col < img.cols; col++) {
            unsigned short value = img.at<unsigned short>(row, col);
            // if (value >= threshold) {
            if (value & MASK) {
                img_ret.at<unsigned char>(row, col) = 255;
                value &= ~MASK;
                img.at<unsigned short>(row, col) = value;
            } else
                img_ret.at<unsigned char>(row, col) = 0;
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Invalid Number of Arguments!\nUsage:\n");
        fprintf(stderr, "<Executable Name> <input image path> \n");
        return EXIT_FAILURE;
    }

    cv::Mat img_src, img_dst, img_dat, img_gld;

    InVideoStrm_t src_axi;
    OutVideoStrm_t dst_axi;

    // read input image
    img_src = cv::imread(argv[1], -1);

    if (img_src.data == NULL) {
        fprintf(stderr, "Cannot open image at %s\n", argv[1]);
        return EXIT_FAILURE;
    }

    std::cout << "Input image height : " << img_src.rows << std::endl;
    std::cout << "Input image width  : " << img_src.cols << std::endl;
    std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC) << std::endl;
    std::cout << "Input Image Channels:" << XF_CHANNELS(XF_SRC_T, XF_NPPC) << std::endl;
    std::cout << "NPPC:" << XF_NPPC << std::endl;
    std::cout << "IN DEPTH:" << img_src.depth() << std::endl;

    img_dst.create(img_src.rows, img_src.cols, CV_16UC1);
    img_dat.create(img_src.rows, img_src.cols, CV_8UC1);
    img_gld.create(img_src.rows, img_src.cols, CV_8UC1);
    imwrite("input.png", img_src);

    unsigned char ctrl = CTRL;
    unsigned short threshold = THRESHOLD;

    GrayMat2AXIvideo(img_src, src_axi);

    // Call IP Processing function
    ALPD_accel(src_axi, dst_axi, (ap_uint<OUTPUT_PTR_WIDTH>*)img_dat.data, img_src.cols, img_src.rows, threshold, ctrl);

    // Convert processed image back to CV image, then to XVID image
    GrayAXIvideo2Mat(dst_axi, img_dst);

    std::cout << "OUT DEPTH:" << img_dst.depth() << std::endl;
    std::cout << "DAT DEPTH:" << img_dat.depth() << std::endl;
    imwrite("output.png", img_dst);
    imwrite("img_dat.png", img_dat);

    if (ctrl) {
        // ALPD Detector cpp for compare results
        ALPD_Detector(img_src, img_gld, threshold);
        std::cout << "GOLD DEPTH:" << img_gld.depth() << std::endl;
        imwrite("gold.png", img_gld);
    }

    if (cv::sum(img_dst != img_src) != cv::Scalar(0, 0, 0, 0)) {
        fprintf(stderr, "ERROR: Test Failed - Destination file not equal to source.\n ");
        return EXIT_FAILURE;
    }

    if (cv::sum(img_dat != img_gld) != cv::Scalar(0, 0, 0, 0)) {
        fprintf(stderr, "ERROR: Test Failed - Gold image not equal to accel.\n ");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
