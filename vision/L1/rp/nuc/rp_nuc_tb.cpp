/*
 * Testbench
 */

#include "common/xf_headers.hpp"
#include "rp_nuc_types.h"


#define CTRL 1

using namespace std;

/*********************************************************************************
 * Function:    GrayMat2AXIvideo
 * Description: 16 bit Mat to stream
 **********************************************************************************/
static void GrayMat2AXIvideo(cv::Mat& img, VideoStrm_t& AXI_video_strm) {
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
static void GrayAXIvideo2Mat(VideoStrm_t& AXI_video_strm, cv::Mat& img) {
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

void SetOffset(cv::Mat& img) {
    int value = -100;
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            // value = (rand() % 2000) - 500;
            img.at<int>(i, j) = value++;
        }
    }
}

void SetGain(cv::Mat& img) {
    int value = 1024;
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            // value = (rand() % 2000);
            img.at<int>(i, j) = value++;
        }
    }
}

void SetGolden(cv::Mat& src, cv::Mat& dest, cv::Mat& gain, cv::Mat& offset) {
    for (int row = 0; row < src.rows; row++) {
        for (int col = 0; col < src.cols; col++) {
            int16_t _src = src.at<uint16_t>(row, col);
            int _gain = gain.at<int>(row, col);
            int _offset = offset.at<int>(row, col);
            int val = ((_src * _gain) >> 10) + _offset;
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

    VideoStrm_t src_axi;
    VideoStrm_t dst_axi;
    cv::Mat img_src, img_dst, gain, offset, img_gld;
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

    gain.create(img_src.rows, img_src.cols, CV_32SC1);
    offset.create(img_src.rows, img_src.cols, CV_32SC1);
    img_gld.create(img_src.rows, img_src.cols, CV_16UC1);
    img_dst.create(img_src.rows, img_src.cols, CV_16UC1);

    SetGain(gain);
    SetOffset(offset);
    SetGolden(img_src, img_gld, gain, offset);

    // Mat to stream
    GrayMat2AXIvideo(img_src, src_axi);

    // Call IP Processing function
    NUC_accel(src_axi, dst_axi, (ap_uint<INPUT_PTR_WIDTH>*)gain.data, (ap_uint<INPUT_PTR_WIDTH>*)offset.data, img_src.cols, img_src.rows, CTRL);

    // Convert processed image back to CV image
    GrayAXIvideo2Mat(dst_axi, img_dst);


    if (cv::sum(img_dst != img_gld) != cv::Scalar(0, 0, 0, 0)) {
        fprintf(stderr, "ERROR: Test Failed - Gold image not equal to accel.\n ");
        return EXIT_FAILURE;
    }

    printf("-: Simulation done!\n");
    return EXIT_SUCCESS;
}
