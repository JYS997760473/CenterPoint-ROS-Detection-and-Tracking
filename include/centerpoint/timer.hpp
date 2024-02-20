/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
 
#ifndef TIMER_HPP
#define TIMER_HPP

#include "centerpoint/common.h"

class EventTimer{
public:
    EventTimer(){
        checkCudaErrors(cudaEventCreate(&begin_));
        checkCudaErrors(cudaEventCreate(&end_));
    }

    virtual ~EventTimer(){
        checkCudaErrors(cudaEventDestroy(begin_));
        checkCudaErrors(cudaEventDestroy(end_));
    }

    void start(cudaStream_t stream){
        checkCudaErrors(cudaEventRecord(begin_, stream));
    }

    float stop(const char* prefix = "timer", bool print = true){
        float times = 0;
        checkCudaErrors(cudaEventRecord(end_, stream_));
        checkCudaErrors(cudaEventSynchronize(end_));
        checkCudaErrors(cudaEventElapsedTime(&times, begin_, end_));
        if(print) printf("[TIME] %s:\t\t%.5f ms\n", prefix, times);
        return times;
    }

private:
    cudaStream_t stream_ = nullptr;
    cudaEvent_t begin_ = nullptr, end_ = nullptr;
};




#endif // TIMER_HPP