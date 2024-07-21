/*
 * File:     xf_alpd_tb.cpp
 * Notes:
 *
 * Author:   Engr. Max Parker
 * Created:  Thu Jul 11 2024
 *
 * Copyright (C) 2024 RP Optical Lab
 */

#include "common/xf_headers.hpp"
#include "xf_alpd_types.h"

using namespace std;

void ALPD_Detector(cv::Mat& img, cv::Mat& img_ret) {
    unsigned int MASK = 0x2000;
    for (int row = 0; row < img.rows; row++) {
        for (int col = 0; col < img.cols; col++) {
            unsigned short value = img.at<unsigned short>(row, col);
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
    printf("-: Start C++ simulation\n");

    cv::Mat img_src, img_ltm, img_dst, img_gld;
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

    img_ltm.create(img_src.rows, img_src.cols, CV_8UC1);
    img_dst.create(img_src.rows, img_src.cols, CV_16UC1);
    img_gld.create(img_src.rows, img_src.cols, CV_8UC1);


    // Call IP Processing function
    ALPD_Accel((ap_uint<INPUT_PTR_WIDTH>*)img_src.data, (ap_uint<OUTLTM_PTR_WIDTH>*)img_ltm.data,
               (ap_uint<OUTPUT_PTR_WIDTH>*)img_dst.data, img_src.cols, img_src.rows);

    ALPD_Detector(img_src, img_gld);

    printf("-: Compare Gold image with accel output\n");
    if (cv::sum(img_ltm != img_gld) != cv::Scalar(0, 0, 0, 0)) {
        fprintf(stderr, "ERROR: Test Failed - Gold image not equal to accel.\n ");
        return EXIT_FAILURE;
    }
    printf("-: Compare Destination image with accel output\n");
    if (cv::sum(img_dst != img_src) != cv::Scalar(0, 0, 0, 0)) {
        fprintf(stderr, "ERROR: Test Failed - Destination file not equal to accel.\n ");
        return EXIT_FAILURE;
    }

    printf("-: Simulation done!\n");
    return EXIT_SUCCESS;
}
