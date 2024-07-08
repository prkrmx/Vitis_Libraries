/*
 * Testbench
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
    uint16_t mmin = 0, mmax = 0;
    uint64_t sum = 0;
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

    double min = 0, max = 0;
    cv::Scalar sclr = cv::mean(img_src);
    cv::minMaxLoc(img_src, &min, &max);
    uint32_t mean = (uint32_t)sclr[0];
    printf("-: Input image min: %d, max: %d, mean %d\n", (uint32_t)min, (uint32_t)max, mean);

    // Mat to stream
    // GrayMat2AXIvideo(img_src, src_axi);

    // Call IP Processing function
    Histogram_accel((ap_uint<INPUT_PTR_WIDTH>*)img_src.data, histogram, img_src.cols, img_src.rows, &mmin, &mmax, &sum);
    printf("-: Accel image min: %d, max: %d, mean %d\n", mmin, mmax, sum / (img_src.cols * img_src.rows));

    // Compare histogram results
    for (int cnt = 0; cnt < SIZE; cnt++) {
        uint32_t val = (uint32_t)hist_ocv.at<float>(cnt);
        if (val != histogram[cnt]) {
            fprintf(stderr, "-: Failed histogram test.\n ");
            return EXIT_FAILURE;
        }
    }

    // Compare min, max, mean results
    if (min != mmin || max != mmax || mean != (sum / (img_src.cols * img_src.rows))) {
        fprintf(stderr, "-: Failed min, max, mean test.\n ");
        return EXIT_FAILURE;
    }

    uint32_t hist_w = SIZE, hist_h = 2000;
    uint32_t bin_w = cvRound((double)hist_w / histSize);

    cv::Mat hist_mat(hist_h, hist_w, CV_8UC1, cv::Scalar(0, 0, 0));

    for (int i = 1; i < SIZE; i++) {
        cv::line(hist_mat, 
                 cv::Point(bin_w * (i), hist_h),
                 cv::Point(bin_w * (i), hist_h - (uint32_t)hist_ocv.at<float>(i)), 
                 cv::Scalar(120, 120, 120), 2, 8, 0);
    }

    imwrite("histogram.png", hist_mat);

    printf("-: Simulation done!\n");
    return EXIT_SUCCESS;
}
