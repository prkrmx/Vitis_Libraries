#
#

source settings.tcl

set PROJ "alpd_mm.prj"
set SOLN "sol1"

if {![info exists CLKP]} {
  set CLKP 3.3
}

open_project -reset $PROJ

add_files "${XF_PROJ_ROOT}/L1/rp/alpd_mm/xf_alpd_accel.cpp" -cflags " -I ${XF_PROJ_ROOT}/L1/rp/alpd_mm/build -I${XF_PROJ_ROOT}/L1/include -I ./ -D__SDSVHLS__ -std=c++0x" -csimflags " -I ${XF_PROJ_ROOT}/L1/rp/alpd_mm/build -I${XF_PROJ_ROOT}/L1/include -I ./ -D__SDSVHLS__ -std=c++0x"
add_files -tb "${XF_PROJ_ROOT}/L1/rp/alpd_mm/xf_alpd_tb.cpp" -cflags " -I ${XF_PROJ_ROOT}/L1/rp/alpd_mm/build -I${OPENCV_INCLUDE} -I${XF_PROJ_ROOT}/L1/include -I ./ -D__SDSVHLS__ -std=c++0x" -csimflags " -I ${XF_PROJ_ROOT}/L1/rp/alpd_mm/build -I${XF_PROJ_ROOT}/L1/include -I ./ -D__SDSVHLS__ -std=c++0x"
set_top ALPD_Accel

open_solution -reset $SOLN




set_part $XPART
create_clock -period $CLKP

if {$CSIM == 1} {
  csim_design -ldflags "-L ${OPENCV_LIB} -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_flann -lopencv_features2d" -argv " ${XF_PROJ_ROOT}/data/alpd_2.tiff "
}

if {$CSYNTH == 1} {
  csynth_design
}

if {$COSIM == 1} {
  cosim_design -ldflags "-L ${OPENCV_LIB} -lopencv_imgcodecs -lopencv_imgproc -lopencv_core -lopencv_highgui -lopencv_flann -lopencv_features2d" -argv " ${XF_PROJ_ROOT}/data/alpd_2.tiff "
}

if {$VIVADO_SYN == 1} {
  export_design -flow syn -rtl verilog
}

if {$VIVADO_IMPL == 1} {
  export_design -flow impl -rtl verilog -format ip_catalog -output /home/max/Dropbox/projects/swir/hw.swir/hw.swir.ip_hls/alpd_mm
}

exit