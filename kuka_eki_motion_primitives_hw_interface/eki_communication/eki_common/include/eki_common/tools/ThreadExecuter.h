// Copyright (c) 2025, H-KA Hochschule Karlsruhe - University of Applied Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Moritz Weisenböhler

#pragma once

#include <thread>

#include <eki_common/default.h>

class ThreadExecuter
{
public:
    ThreadExecuter(const std::string &filepath) : filepath_(filepath) {}
    ~ThreadExecuter() {}

    std::string filepath(){ return filepath_; }
    bool is_running() { return thread_handle_ > 0; }

    bool run();
    bool cancel();

private:
    std::string filepath_;

    pthread_t thread_handle_ = 0;
};