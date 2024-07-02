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
#include "rp_histogram_types.h"


#define SIZE 8192

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

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Invalid Number of Arguments!\nUsage:\n");
        fprintf(stderr, "<Executable Name> <input image path> \n");
        return EXIT_FAILURE;
    }
    printf("-: Start C++ simulation\n");

    cv::Mat img_src, hist_ocv;
    InVideoStrm_t src_axi;
    int histSize = SIZE;
    float range[] = {0, SIZE};
    const float* histRange = {range};
    unsigned int* histogram = (unsigned int*)calloc(SIZE, sizeof(unsigned int));
    // unsigned int* histogram = (unsigned int*)malloc(SIZE * sizeof(unsigned int));

    // read input image
    img_src = cv::imread(argv[1], -1);

    if (img_src.data == NULL) {
        fprintf(stderr, "Cannot open image at %s\n", argv[1]);
        return EXIT_FAILURE;
    }

    printf("-: Input image height: %d\n", img_src.rows);
    printf("-: Input image width: %d\n", img_src.cols);
    printf("-: Input image bit depth: %d\n", XF_DTPIXELDEPTH(XF_SRC_T, XF_NPPC));
    printf("-: Input image channels: %d\n", XF_CHANNELS(XF_SRC_T, XF_NPPC));
    printf("-: NPPC: %d\n", XF_NPPC);
    printf("-: DEPTH: %d\n", img_src.depth());

    imwrite("input.png", img_src);

    cv::calcHist(&img_src, 1, 0, cv::Mat(), hist_ocv, 1, &histSize, &histRange, 1, 0);

    double min, max, mean;
    cv::Scalar sclr = cv::mean(img_src);
    cv::minMaxLoc(img_src, &min, &max);

    printf("-: Input image min: %d, max: %d, mean %d\n", (uint32_t)min, (uint32_t)max, (uint32_t)sclr[0]);

    // Mat to stream
    GrayMat2AXIvideo(img_src, src_axi);

    // Call IP Processing function
    Histogram_accel(src_axi, histogram, img_src.cols, img_src.rows);

    // Compare results
    for (int cnt = 0; cnt < SIZE; cnt++) {
        uint32_t val = (uint32_t)hist_ocv.at<float>(cnt);
        if (val != histogram[cnt]) {
            fprintf(stderr, "-: Failed.\n ");
            return EXIT_FAILURE;
        }
    }

    uint32_t hist_w = 8192, hist_h = 2000;
    uint32_t bin_w = cvRound((double)hist_w / histSize);

    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0, 0, 0));

    for (int i = 1; i < SIZE; i++) {
        cv::line(histImage, 
                 cv::Point(bin_w * (i), hist_h),
                 cv::Point(bin_w * (i), hist_h - (uint32_t)hist_ocv.at<float>(i)), 
                 cv::Scalar(80, 80, 80), 2, 8, 0);
    }

    imwrite("histogram.png", histImage);

    printf("-: Simulation done!\n");
    return EXIT_SUCCESS;
}
