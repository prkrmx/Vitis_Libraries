#ifndef IFFT_Stream_cc0e4fa5_GRAPH_H_
#define IFFT_Stream_cc0e4fa5_GRAPH_H_

#include <adf.h>
#include "fft_ifft_dit_1ch_graph.hpp"


class IFFT_Stream_cc0e4fa5 : public adf::graph {
public:
  // ports
  template <typename dir>
  using ssr_port_array = std::array<adf::port<dir>, 2>;

  ssr_port_array<input> in;
  ssr_port_array<output> out;


  xf::dsp::aie::fft::dit_1ch::fft_ifft_dit_1ch_graph<
    cint16, // TT_DATA
    cint16, // TT_TWIDDLE
    64, // TP_POINT_SIZE
    0, // TP_FFT_NIFFT
    0, // TP_SHIFT
    1, // TP_CASC_LEN
    0, // TP_DYN_PT_SIZE
    64, // TP_WINDOW_VSIZE
    1, // TP_API
    0, // TP_PARALLEL_POWER
    0, // TP_USE_WIDGETS
    4, // TP_RND
    0 // TP_SAT
  > fft_graph;

  IFFT_Stream_cc0e4fa5() : fft_graph() {
    for (int i=0; i < 2; i++) {
      adf::connect<> net_in(in[i], fft_graph.in[i]);
      adf::connect<> net_out(fft_graph.out[i], out[i]);
    }
  }
};


#endif // IFFT_Stream_cc0e4fa5_GRAPH_H_