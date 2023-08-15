/*
 * Copyright (C) 2019-2022, Xilinx, Inc.
 * Copyright (C) 2022-2023, Advanced Micro Devices, Inc.
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

#include <stdint.h>
#include "xf_data_mover/load_master_to_stream.hpp"
#include "xf_data_mover/get_count.hpp"
#define NUM_CH 2 


extern "C" void m2s_x2(
    // 0
    ap_uint<64>* din2a,
    hls::stream<ap_axiu<64, 0, 0, 0> >& sout2a,
    uint64_t sz0,

    // 1
    ap_uint<32>* din2b,
    hls::stream<ap_axiu<32, 0, 0, 0> >& sout2b,
    uint64_t sz1,

    uint64_t* m2s_counter) 
{
    using namespace xf::data_mover;

    ; // clang-format off
#pragma HLS interface m_axi offset=slave bundle=gmem0 port=din2a \
    max_read_burst_length=32 num_read_outstanding=4 latency=128
#pragma HLS interface s_axilite bundle=control port=din2a
#pragma HLS interface axis port=sout2a
#pragma HLS interface s_axilite bundle=control port=sz0

#pragma HLS interface m_axi offset=slave bundle=gmem1 port=din2b \
    max_read_burst_length=32 num_read_outstanding=4 latency=128
#pragma HLS interface s_axilite bundle=control port=din2b
#pragma HLS interface axis port=sout2b
#pragma HLS interface s_axilite bundle=control port=sz1

#pragma HLS interface m_axi offset=slave bundle=gmem_counter port=m2s_counter\
    max_write_burst_length=32 num_write_outstanding=4 latency=128
#pragma HLS interface s_axilite bundle=control port=m2s_counter

#pragma HLS interface s_axilite bundle=control port=return
    ; // clang-format on

    hls::stream<uint64_t> counter_str[NUM_CH];
    hls::stream<ap_uint<1> > mvld_str[NUM_CH];
    uint64_t cnt_data[NUM_CH];
#pragma HLS ARRAY_PARTITION variable=cnt_data complete 

#pragma HLS dataflow

    loadMasterToStream(din2a, sout2a, sz0, mvld_str[0], &cnt_data[0]);
    get_count(mvld_str[0], counter_str[0]);

    loadMasterToStream(din2b, sout2b, sz1, mvld_str[1], &cnt_data[1]);
    get_count(mvld_str[1], counter_str[1]);

    write_count(counter_str, cnt_data, m2s_counter, NUM_CH);
}