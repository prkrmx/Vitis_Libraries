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


#define XF_WIDTH 1280  // MAX_COLS
#define XF_HEIGHT 1024 // MAX_ROWS

#define XF_SRC_T XF_16UC1
#define XF_DST_T XF_16UC1

#define XF_CV_DEPTH_INP 2
#define XF_CV_DEPTH_OUT 2

#define NO 0 // Normal Operation
#define RO 0 // Resource Optimized

#if NO 
// Normal Operation
#define XF_NPPC XF_NPPC1
#define INPUT_PTR_WIDTH 16
#define OUTPUT_PTR_WIDTH 16
#else
// #endif
// #if RO 
// Resource Optimized
#define XF_NPPC XF_NPPC8
#define INPUT_PTR_WIDTH 128
#define OUTPUT_PTR_WIDTH 128
#endif