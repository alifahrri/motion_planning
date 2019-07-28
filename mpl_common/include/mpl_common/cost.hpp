#ifndef COST_HPP
#define COST_HPP

#include <cstdio>

namespace mpl {
    /**
     * @brief : generic class wrapper 
     * @tparam :
     * - Scalar : cost representation
     * - State : state representation
     * - Solver : cost solver representation
     * @requirements :
     * - Solver should have public member function cost and return Scalar
     * - Solver should have public member type State
     * - State should be convertible to Solver::State
     */
    template <typename Scalar, typename State, typename Solver>
    struct Cost {
        /* member types */
        using cost_type     = Scalar;
        using state_type    = State;
        using solver_type   = Solver;
        
        Cost(Solver &solver) 
            : solver(solver)
        {}
        Scalar operator()(const State &s0, const State &s1)
        {
            using solver_state_t = typename Solver::State;
            return std::get<1>(solver.cost(
                solver_state_t(s0),
                solver_state_t(s1)
            ));
        }
        Solver &solver;
    };
} // namespace mpl

#endif // COST_HPP