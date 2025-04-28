#include <eki_common/core/Chrono.h>

Chrono Chrono::global_instance = Chrono();

std::string Chrono::start()
{
    std::string key;

    for (size_t i = 1;; ++i)
    {
        key = "entry_" + std::to_string(i);

        if (entries.count(key) == 0)
        {
            break;
        }
    }

    return start(key);
}

std::string Chrono::start(std::string key)
{
    entries[key] = ChronoEntry();

    entries[key].start();

    return key;
}

int64_t Chrono::stop(std::string key)
{
    if (entries.count(key) == 1)
    {
        entries[key].stop();

        return entries[key].duration();
    }

    return -1;
}
