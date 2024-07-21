/*
 * File:     xf_alpd_config.h
 * Notes:    
 *
 * Author:   Engr. Max Parker
 * Created:  Thu Jul 11 2024
 *
 * Copyright (C) 2024 RP Optical Lab
 */

#define XF_WIDTH 1280  // MAX_COLS
#define XF_HEIGHT 1024 // MAX_ROWS

#define XF_SRC_T XF_16UC1
#define XF_LTM_T XF_8UC1
#define XF_DST_T XF_16UC1

#define XF_CV_DEPTH_INP 2
#define XF_CV_DEPTH_LTM 0
#define XF_CV_DEPTH_OUT 2

#define NO 0 // Normal Operation
#define RO 1 // Resource Optimized

#if NO 
// Normal Operation
#define XF_NPPC XF_NPPC1
#define INPUT_PTR_WIDTH 16
#define OUTLTM_PTR_WIDTH 8
#define OUTPUT_PTR_WIDTH 16
#endif
#if RO 
// Resource Optimized
#define XF_NPPC XF_NPPC8
#define INPUT_PTR_WIDTH 128
#define OUTLTM_PTR_WIDTH 64
#define OUTPUT_PTR_WIDTH 128
#endif