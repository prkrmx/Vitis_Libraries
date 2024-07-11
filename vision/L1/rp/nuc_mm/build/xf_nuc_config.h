/*
 * File:     xf_nuc_config.h
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
#define XF_TBG_T XF_16UC1
#define XF_TBO_T XF_16SC1
#define XF_DST_T XF_16UC1

#define XF_CV_DEPTH_INP 2
#define XF_CV_DEPTH_TBG 2
#define XF_CV_DEPTH_TBO 3
#define XF_CV_DEPTH_OUT 2

#define NO 0 // Normal Operation
#define RO 0 // Resource Optimized

#if NO 
// Normal Operation
#define XF_NPPC XF_NPPC1
#define INPUT_PTR_WIDTH 16
#define IN_TBG_PTR_WIDTH 16
#define IN_TBO_PTR_WIDTH 16
#define OUTPUT_PTR_WIDTH 16
#else
// #endif
// #if RO 
// Resource Optimized
#define XF_NPPC XF_NPPC8
#define INPUT_PTR_WIDTH 128
#define IN_TBG_PTR_WIDTH 128
#define IN_TBO_PTR_WIDTH 128
#define OUTPUT_PTR_WIDTH 128
#endif