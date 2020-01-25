#ifndef CORE_GOALCHECKER_HPP
#define CORE_GOALCHECKER_HPP

#include <type_traits>

namespace mpl {
    namespace core {
        namespace detail {
            template <typename T, typename = void>
            struct has_checker_var : std::false_type {};
            
            template <typename T>
            struct has_checker_var<T,std::void_t<
                decltype(std::declval<T>().checker)
            > /* void_t */ > : std::true_type {};
        } // namespace detail
        
        /**
         * @brief           : crtp base for goal checker
         * @tparam          : T derived class
         * @requirements    : 
         * - T::state_type is defined       (mandatory)
         * - T has member variable goal     (mandatory)
         * - T has member variable checker  (optional)
         * - T::checker has operator() takes T::state_type returns boolean
         */
        template <typename T>
        struct BaseGoalChecker {
            
            /* TODO : should this operation declared as const (?) */
            /**
             * @brief   : provides convinient calls via operator(), 
             *  if T has checker member variable, calls T::checker(state,goal),
             *  performs goal == state otherwise
             * @tparam  : state_t represents the checked state type
             */
            template <typename state_t>
            bool operator()(const state_t &state) {
                using state_type = typename T::state_type;
                if constexpr (detail::has_checker_var<T>::value) {
                    return static_cast<T*>(this)->checker(
                        static_cast<const state_type&>(state),
                        static_cast<const state_type>(
                            static_cast<T*>(this)->goal
                        )
                    );
                } else {
                    return static_cast<T*>(this)->goal == state;
                }
            }
            
            /**
             * @brief   : set goal
             * @tparam  : state_t represents state type
             * @requirements    :
             * - state_t is assignable to T::goal
             */
            template <typename state_t>
            bool set_goal(const state_t &state) {
                using state_type = typename T::state_type;
                                            // should we staticcally cast to T::state_type ?
                static_cast<T*>(this)->goal = static_cast<const state_type&>(state);
            }
        };
    }
} // namespace mpl
#endif // CORE_GOALCHECKER_HPP