/*
 * Copyright 2022 Xilinx, Inc.
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
#include "xf_channel_extract_tb_config.h"
#include "xcl2.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Invalid Number of Arguments!\nUsage:\n");
        fprintf(stderr, "<Executable Name> <Input image>\n");
        return -1;
    }

    cv::Mat in_src, in_brga, in_accel, out_img, in_ref;

    // read image
    in_src = cv::imread(argv[1], 1);
    in_src.convertTo(in_ref, CV_IN_TYPE);

    if (in_src.data == NULL) {
        fprintf(stderr, "Cannot open image \n");
        return 0;
    }

    out_img.create(in_src.rows, in_src.cols, CV_OUT_TYPE);

    if (RGB) {
        in_ref.copyTo(in_accel);
    } else if (RGBA) {
        cv::cvtColor(in_ref, in_accel, cv::COLOR_BGR2BGRA);
    }

    uint16_t channel = XF_EXTRACT_CH_R;
    int in_bits;
    if (T_16U) {
        in_bits = 2;
    } else {
        in_bits = 1;
    }

    /////////////////////////////////////// CL ////////////////////////
    int height = in_accel.rows;
    int width = in_accel.cols;
    std::cout << "Input image height : " << height << std::endl;
    std::cout << "Input image width  : " << width << std::endl;

    std::vector<cl::Device> devices = xcl::get_xil_devices();
    cl::Device device = devices[0];
    cl::Context context(device);

    cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE);
    std::cout << "Input Image Bit Depth:" << XF_DTPIXELDEPTH(IN_TYPE, NPPCX) << std::endl;
    std::cout << "Input Image Channels:" << XF_CHANNELS(IN_TYPE, NPPCX) << std::endl;
    std::cout << "NPPC:" << NPPCX << std::endl;

    std::string device_name = device.getInfo<CL_DEVICE_NAME>();
    std::string binaryFile = xcl::find_binary_file(device_name, "krnl_channelextract");
    cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
    devices.resize(1);
    cl::Program program(context, devices, bins);
    cl::Kernel krnl(program, "channel_extract_accel");

    std::vector<cl::Memory> inBuf_rgba, outBuf_gray;
    cl::Buffer imageToDevicergba(context, CL_MEM_READ_ONLY, (in_accel.rows * in_accel.cols * 4 * in_bits));
    cl::Buffer imageFromDevicegray(context, CL_MEM_WRITE_ONLY, (in_accel.rows * in_accel.cols * 1 * in_bits));

    printf("finished buffer creation task\n");

    inBuf_rgba.push_back(imageToDevicergba);
    outBuf_gray.push_back(imageFromDevicegray);

    /* Copy input vectors to memory */
    q.enqueueWriteBuffer(imageToDevicergba, CL_TRUE, 0, (in_accel.rows * in_accel.cols * 4 * in_bits),
                         in_accel.data); /* 0 means from host*/
    printf("finished enqueueing task\n");

    // Set the kernel arguments
    krnl.setArg(0, imageToDevicergba);
    krnl.setArg(1, imageFromDevicegray);
    krnl.setArg(2, channel);
    krnl.setArg(3, height);
    krnl.setArg(4, width);

    printf("finished setting kernel arguments\n");

    // Profiling Objects
    cl_ulong start = 0;
    cl_ulong end = 0;
    double diff_prof = 0.0f;
    cl::Event event_sp;
    printf("started kernel execution\n");
    // Launch the kernel
    q.enqueueTask(krnl, NULL, &event_sp);
    clWaitForEvents(1, (const cl_event*)&event_sp);

    printf("finished kernel execution\n");
    event_sp.getProfilingInfo(CL_PROFILING_COMMAND_START, &start);
    event_sp.getProfilingInfo(CL_PROFILING_COMMAND_END, &end);
    diff_prof = end - start;
    std::cout << (diff_prof / 1000000) << "ms" << std::endl;

    q.enqueueReadBuffer(imageFromDevicegray, CL_TRUE, 0, (in_accel.rows * in_accel.cols * in_bits), out_img.data);
    q.finish();
    printf("write output buffer\n");
    /////////////////////////////////////// end of CL /////////////////////

    cv::imwrite("hls_out.png", out_img);
    std::vector<cv::Mat> bgr_planes;

    // call OpenCV function
    cv::split(in_ref, bgr_planes);
    // write output and OpenCV reference image
    cv::imwrite("out_ocv.png", bgr_planes[2]);

    cv::Mat diff;
    diff.create(in_ref.rows, in_ref.cols, CV_OUT_TYPE);

    // Check with the correct channel. Keep 2 for R, 1 for G and 0 for B in index of bgr_planes
    cv::absdiff(bgr_planes[2], out_img, diff);
    cv::imwrite("diff.jpg", diff);

    // Find minimum and maximum differences.
    double minval = 256, maxval = 0;
    int cnt = 0;
    for (int i = 0; i < diff.rows; i++) {
        for (int j = 0; j < diff.cols; j++) {
            unsigned char v = diff.at<unsigned char>(i, j);
            if (v > 0) cnt++;
            if (minval > v) minval = v;
            if (maxval < v) maxval = v;
        }
    }
    float err_per = 100.0 * (float)cnt / (in_ref.rows * in_ref.cols);

    std::cout << "\tMinimum error in intensity = " << minval << std::endl;
    std::cout << "\tMaximum error in intensity = " << maxval << std::endl;
    std::cout << "\tPercentage of pixels above error threshold = " << err_per << std::endl;

    if (err_per > 0.0f) {
        fprintf(stderr, "ERROR: Test Failed.\n ");
        return -1;
    } else
        std::cout << "Test Passed " << std::endl;

    return 0;
}
