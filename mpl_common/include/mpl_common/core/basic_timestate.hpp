#ifndef BASIC_TIMESTATE_HPP
#define BASIC_TIMESTATE_HPP

#include <type_traits>
#include "traits.hpp"

namespace mpl {
    namespace core {
        namespace detail {
            /* helper function for time state */
            auto get_state(auto &tuple) 
                -> decltype((tuple.state())) 
            { 
                return tuple.state();
            }
            auto get_time(auto &tuple) 
                -> decltype((tuple.time())) 
            { 
                return tuple.time(); 
            }
            auto get_state(auto &tuple) 
                -> decltype((std::get<1>(tuple))) 
            { 
                return std::get<1>(tuple); 
            }
            auto get_time(auto &tuple) 
                -> decltype((std::get<0>(tuple))) 
            { 
                return std::get<0>(tuple);
            }
            auto get_state(const auto &tuple) 
                -> decltype((tuple.state())) 
            { 
                return tuple.state();
            }
            auto get_time(const auto &tuple) 
                -> decltype((tuple.time())) 
            { 
                return tuple.time(); 
            }
            auto get_state(const auto &tuple) 
                -> decltype((std::get<1>(tuple))) 
            { 
                return std::get<1>(tuple); 
            }
            auto get_time(const auto &tuple) 
                -> decltype((std::get<0>(tuple))) 
            { 
                return std::get<0>(tuple);
            }
        } // namespace detail
        
        template <typename T>
        struct BaseTimeState
        {
            auto state() const 
            {
                return detail::get_state(static_cast<const T*>(this)->tuple);
            }
            auto time() const 
            {
                return detail::get_time(static_cast<const T*>(this)->tuple);
            }
            auto state() 
            {
                return detail::get_state(static_cast<T*>(this)->tuple);
            }
            auto time() 
            {
                return detail::get_time(static_cast<T*>(this)->tuple);
            }
        };
        
    } // namespace core
} // namespace mpl

#endif // BASIC_TIMESTATE_HPP