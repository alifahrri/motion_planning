#ifndef MPL_GOAL_HPP
#define MPL_GOAL_HPP

#include <type_traits>
#include "core/basic_goal.hpp"

namespace mpl {
    
    /**
     * @brief   : generic class for goal checking purpose
     * @tparam  :
     * - State, goal state type
     * - Checker (optional), definition of additional checker procedure
     *   if other than void, checker(state,goal) will be called, 
     *   i.e. you want to specify goal as area instead of point (State)
     */
    template <typename State, typename Checker = void>
    struct GoalChecker : core::BaseGoalChecker<GoalChecker<State,Checker>> {
        using state_type = State;
        /* TODO : move (Checker&&) instead of taking reference to Checker (?) */
        GoalChecker(Checker &checker) : checker(checker) {}
        Checker checker;
        State goal;
    };
    
    /**
     * @brief   : partial specialization of generic goal checker, 
     *  performs simple check, e.g. goal == state
     * @tparam  : 
     * - State, goal state type
     */
    template <typename State>
    struct GoalChecker<State,void> : core::BaseGoalChecker<GoalChecker<State,void>> {
        using state_type = State;
        State goal;
    };
    
    /**
     * @brief   : dummy goal, always return false
     */
    struct DummyCheck {
        constexpr bool operator()(auto,auto) const {
            return false;
        }
        constexpr bool operator()(auto) const {
            return false;
        }
    };
    
    template <typename State>
    struct GoalChecker<State,DummyCheck> : core::BaseGoalChecker<GoalChecker<State,DummyCheck>> {
        DummyCheck checker;
        State goal;
    };
    
} // namespace mpl

#endif // MPL_GOAL_HPP