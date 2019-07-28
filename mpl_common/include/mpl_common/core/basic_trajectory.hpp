#ifndef BASIC_TRAJECTORY_HPP
#define BASIC_TRAJECTORY_HPP

#include <type_traits>
#include "traits.hpp"
#include "core/basic_timestate.hpp"

namespace mpl {
    namespace core {
        
        /**
         * @brief : crtp base for trajectory, encapsulate 'trajectory' so that T is iterable
         * @tparam : T
         * @requirements : 
         *  - T has member variable named trajectory
         *  - T::trajectory defines value_type
         *  - value_type of T defines time_type
         *  - value_type of T defines state_type
         *  - value_type of T has member funtion state and time
         */
        template <typename T>
        struct BaseIterableTrajectory {
            /**
            * @brief : access to placeholder
            * @param : i index of the trajectory
            */
            decltype(auto) operator[](size_t i) {
                return (static_cast<T*>(this)->trajectory[i]);
            }
            decltype(auto) operator[](size_t i) const {
                return (static_cast<const T*>(this)->trajectory[i]);
            }
            decltype(auto) at(size_t i) {
                return (static_cast<T*>(this)->trajectory.at(i));
            }
            decltype(auto) at(size_t i) const {
                return (static_cast<const T*>(this)->trajectory.at(i));
            }
            /**
            * @brief : make this type iterable
            */
            decltype(auto) begin() {
                return (static_cast<T*>(this)->trajectory.begin());
            }
            decltype(auto) begin() const {
                return (static_cast<const T*>(this)->trajectory.begin());
            }
            decltype(auto) end() {
                return (static_cast<T*>(this)->trajectory.end());
            }
            decltype(auto) end() const {
                return (static_cast<const T*>(this)->trajectory.end());
            }
            /**
            * @brief : access to first and last element
            */
            decltype(auto) front() {
                return (static_cast<T*>(this)->trajectory.front());
            }
            decltype(auto) front() const {
                return (static_cast<const T*>(this)->trajectory.front());
            }
            decltype(auto) back() {
                return (static_cast<T*>(this)->trajectory.back());
            }
            decltype(auto) back() const {
                return (static_cast<const T*>(this)->trajectory.back());
            }
            /**
            * @brief : get the size of trajectory container
            * @return : the length of trajectory
            */
            constexpr size_t size() const {
                return std::size(static_cast<const T*>(this)->trajectory);
            }
            
            /**
            * @brief : extract time
            * @return : std::vector< value_type::time_type > if trajectory has resize member function,
            * std::array< value_type::time_type > else
            */
            auto time() const
            {
                using timestate_container_type = std::decay_t<decltype(static_cast<const T*>(this)->trajectory)>;
                using time_type = typename timestate_container_type::value_type::time_type;
                if constexpr (traits::has_resize_op<timestate_container_type>::value) {
                    using time_container_type = std::vector<time_type>;
                    time_container_type ret;
                    ret.resize(static_cast<const T*>(this)->size());
                    for(size_t i=0; i<static_cast<const T*>(this)->size(); i++) 
                        ret.at(i) = static_cast<const T*>(this)->at(i).time();
                    return ret;
                } else {
                    using container_size = std::tuple_size<timestate_container_type>;
                    using time_container_type = std::array<time_type,container_size::value>;
                    time_container_type ret;
                    for(size_t i=0; i<static_cast<const T*>(this)->size(); i++) 
                        ret.at(i) = static_cast<const T*>(this)->at(i).time();
                    return ret;
                }
            }
            /**
            * @brief : extract path
            * @return : std::vector< value_type::state_type > if trajectory has resize member function,
            * std::array< value_type::state_type > else
            */
            auto path() const 
            {
                using timestate_container_type = std::decay_t<decltype(static_cast<const T*>(this)->trajectory)>;
                using state_type = typename timestate_container_type::value_type::state_type;
                if constexpr (traits::has_resize_op<timestate_container_type>::value) {
                    using state_container_type = std::vector<state_type>;
                    state_container_type ret;
                    ret.resize(static_cast<const T*>(this)->size());
                    for(size_t i=0; i<static_cast<const T*>(this)->size(); i++) 
                        ret.at(i) = static_cast<const T*>(this)->at(i).state();
                    return ret;
                } else {
                    using container_size = std::tuple_size<timestate_container_type>;
                    using state_container_type = std::array<state_type,container_size::value>;
                    state_container_type ret;
                    for(size_t i=0; i<static_cast<const T*>(this)->size(); i++) 
                        ret.at(i) = static_cast<const T*>(this)->at(i).state();
                    return ret;
                }
            }
            
            /**
            * @brief : construct Trajectory from trajectory with different type
            * @tparam : trajectory_t type that represents trajectory
            * @requirements :
            * - a call to std::size(trajectory_t{}) is valid
            * - trajectory_t has member function [] that returns time-state representation, say ts_repr_t
            * - a call to mpl::core::detail::get_time(ts_repr_t{}) is valid and its return value is Convertible to time_type
            * - a call to mpl::core::detail::get_state(ts_repr_t{}) is valid and its return value is Convertible to state_type
            */
            template <typename trajectory_t>
            auto copy_from(const trajectory_t &t) 
            {
                using timestate_container_type = std::decay_t<decltype(static_cast<T*>(this)->trajectory)>;
                using timestate_type = typename timestate_container_type::value_type;
                using time_type = typename timestate_type::time_type;
                using state_type = typename timestate_type::state_type;
                if constexpr (traits::has_push_back_op<timestate_container_type>::value) {
                    for (const auto &v : t) {
                        auto tuple = timestate_type(
                            time_type(core::detail::get_time(v)),
                            state_type(core::detail::get_state(v))
                        );
                        static_cast<T*>(this)->push_back(tuple);
                    }
                } else {
                    auto it = std::min(std::size(static_cast<const T*>(this)->trajectory),std::size(t));
                    for (size_t i=0; i<it; i++) {
                        auto tuple = timestate_type(
                            time_type(core::detail::get_time(t[i])),
                            state_type(core::detail::get_state(t[i]))
                        );
                        (*static_cast<T*>(this))[i] = tuple;
                    }
                }
            }
        };
    } // namespace core
} // namespace mpl

#endif // BASIC_TRAJECTORY_HPP