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

#define XF_NPPC XF_NPPC2 // XF_NPPC1 --1PIXEL , XF_NPPC2--2PIXEL ,XF_NPPC4--4 and XF_NPPC8--8PIXEL

#define XF_WIDTH 1280  // MAX_COLS
#define XF_HEIGHT 1024 // MAX_ROWS

#define T_8U 0
#define T_10U 0
#define T_12U 0
#define T_16U 1

#define OUTPUT_PTR_WIDTH 16
#define XF_LTM_T XF_8UC1


#if T_8U
#define XF_SRC_T XF_8UC1
#define XF_DST_T XF_8UC1
#elif T_16U
#define XF_SRC_T XF_16UC1
#define XF_DST_T XF_16UC1
#elif T_10U
#define XF_SRC_T XF_10UC1
#define XF_DST_T XF_10UC1
#elif T_12U
#define XF_SRC_T XF_12UC1
#define XF_DST_T XF_12UC1
#endif

#define XF_CV_DEPTH_INP 2
#define XF_CV_DEPTH_LTM 0
#define XF_CV_DEPTH_OUT 2