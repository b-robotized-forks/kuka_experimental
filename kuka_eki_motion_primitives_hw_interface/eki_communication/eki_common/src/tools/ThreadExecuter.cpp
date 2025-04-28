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

#include <eki_common/tools/ThreadExecuter.h>

#include <signal.h>

bool ThreadExecuter::run()
{
    if (!is_running())
    {
        std::thread thread(system, filepath_.c_str());
        
        thread_handle_ = thread.native_handle();
            
        thread.detach();

        return true;
    }

    return false;
}

bool ThreadExecuter::cancel()
{
    if (is_running())
    {
        // Kill and cancel do not work (only cntl+c)
        int r = pthread_kill(thread_handle_, SIGINT);
        //int r = pthread_cancel(thread_handle_);

        thread_handle_ = 0;

        return true;
    }

    return false;
}