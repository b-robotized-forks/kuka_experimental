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