/*
 * File:     xf_nuc_tb.cpp
 * Notes:    
 *
 * Author:   Engr. Max Parker
 * Created:  Thu Jul 11 2024
 *
 * Copyright (C) 2024 RP Optical Lab
 */


#include "common/xf_headers.hpp"
#include "xf_nuc_types.h"


using namespace std;
#define CTRL 0

void SetTables(cv::Mat& mat_gain, cv::Mat& mat_offset) {
    for (int i = 0; i < mat_gain.rows; i++) {
        for (int j = 0; j < mat_gain.cols; j++) {
            uint16_t gain = (rand() % 2000) + 1024;
            int16_t offset = (rand() % 2000) - 500;
            mat_gain.at<uint16_t>(i, j) = gain;
            mat_offset.at<int16_t>(i, j) = offset;
        }
    }
}

void SetGolden(cv::Mat& src, cv::Mat& dest, cv::Mat& gain, cv::Mat& offset) {
    for (int row = 0; row < src.rows; row++) {
        for (int col = 0; col < src.cols; col++) {
            uint16_t _src = src.at<uint16_t>(row, col);
            uint16_t _gain = gain.at<uint16_t>(row, col);
            int16_t _offset = offset.at<int16_t>(row, col);
            uint16_t val = ((_src * _gain) >> 10) + _offset;
            dest.at<uint16_t>(row, col) = (uint16_t)val;
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

    cv::Mat img_src, img_dst, gain, offset, img_gld;
    uint8_t ctrl = CTRL;
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

    gain.create(img_src.rows, img_src.cols, CV_16UC1);
    offset.create(img_src.rows, img_src.cols, CV_16SC1);
    img_gld.create(img_src.rows, img_src.cols, CV_16UC1);
    img_dst.create(img_src.rows, img_src.cols, CV_16UC1);

    SetTables(gain, offset);
    SetGolden(img_src, img_gld, gain, offset);

    // Call IP Processing function
    NUC2P_Accel((ap_uint<INPUT_PTR_WIDTH>*)img_src.data, (ap_uint<IN_TBG_PTR_WIDTH>*)gain.data,
                (ap_uint<IN_TBO_PTR_WIDTH>*)offset.data, (ap_uint<OUTPUT_PTR_WIDTH>*)img_dst.data, img_src.cols,
                img_src.rows, ctrl);

    if (ctrl) {
        printf("-: DO JOB: Compare Gold image with accel output\n");
        if (cv::sum(img_dst != img_gld) != cv::Scalar(0, 0, 0, 0)) {
            fprintf(stderr, "ERROR: Test Failed - Gold image not equal to accel.\n ");
            return EXIT_FAILURE;
        }
    } else {
        printf("-: COPY: Compare Source image with accel output\n");
        if (cv::sum(img_dst != img_src) != cv::Scalar(0, 0, 0, 0)) {
            fprintf(stderr, "ERROR: Test Failed - Source image not equal to accel.\n ");
            return EXIT_FAILURE;
        }
    }

    printf("-: Simulation done!\n");
    return EXIT_SUCCESS;
}
