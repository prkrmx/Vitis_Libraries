
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

#include <iostream>
#include <stdlib.h>

#include "xf_data_mover/write_result.hpp"

void dut(hls::stream<xf::data_mover::CheckResult::type>& rs1,
         xf::data_mover::CheckResult::type* rm1,
         hls::stream<xf::data_mover::CheckResult::type>& rs2,
         xf::data_mover::CheckResult::type* rm2) {
    xf::data_mover::writeResult(rs1, rm1, rs2, rm2);
}

int main() {
    std::cout << "Testing writeResult..." << std::endl;
    hls::stream<xf::data_mover::CheckResult::type> rs1;
    xf::data_mover::CheckResult::type rm1[1];
    hls::stream<xf::data_mover::CheckResult::type> rs2;
    xf::data_mover::CheckResult::type rm2[1];
    // preparing input streams
    rs1.write(0);
    rs2.write(1);
    // call FPGA
    dut(rs1, rm1, rs2, rm2);
    int nerror = 0;
    // check result
    xf::data_mover::CheckResult::type out = *rm1;
    if (out != 0) {
        nerror++;
    }
    out = *rm2;
    if (out != 1) {
        nerror++;
    }
    if (nerror) {
        std::cout << "FAIL: " << nerror << " errors in 2 channels." << std::endl;
    } else {
        std::cout << "PASS: 2 channels verified." << std::endl;
    }

    return nerror;
}
