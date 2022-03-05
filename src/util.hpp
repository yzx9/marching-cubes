#include <chrono>
#include <functional>
#include <exception>
#include <type_traits>
#include <string>
#include <iostream>
#include <iomanip>

namespace util
{
    void print_duration_info(const std::string &title, std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point stop)
    {
        std::chrono::nanoseconds nanoseconds = stop - start;

        auto hours = std::chrono::duration_cast<std::chrono::hours>(nanoseconds);
        nanoseconds -= hours;

        auto minutes = std::chrono::duration_cast<std::chrono::minutes>(nanoseconds);
        nanoseconds -= minutes;

        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(nanoseconds);
        nanoseconds -= seconds;

        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(nanoseconds);

        std::cout << title << " complete: " << std::endl
                  << "Time taken: " << std::setw(4) << hours.count() << " hours, " << std::endl
                  << "            " << std::setw(4) << minutes.count() << " minutes, " << std::endl
                  << "            " << std::setw(4) << seconds.count() << " seconds, " << std::endl
                  << "            " << std::setw(4) << milliseconds.count() << " milliseconds." << std::endl
                  << std::endl;
    }

    template <typename Func, typename... Args>
    auto run_with_duration(const std::string &title, const Func &func, Args... args)
    {
        if constexpr (std::is_same<std::invoke_result_t<Func, Args...>, void>::value)
        {
            auto start = std::chrono::steady_clock::now();
            func(args...);
            auto stop = std::chrono::steady_clock::now();

            print_duration_info(title, start, stop);
        }
        else
        {
            auto start = std::chrono::steady_clock::now();
            std::invoke_result_t<Func, Args...> result = func(args...);
            auto stop = std::chrono::steady_clock::now();

            print_duration_info(title, start, stop);

            return result;
        }
    }
}
