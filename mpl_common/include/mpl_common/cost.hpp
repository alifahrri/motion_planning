#ifndef COST_HPP
#define COST_HPP

#include <cstdio>

namespace mpl {
    template <typename Scalar, typename State, typename Solver>
    struct Cost {
        /* member types */
        using cost_type = Scalar;
        using state_type = State;
        using solver_type = Solver;
        
        Cost(Solver &solver) 
            : solver(solver)
        {}
        Scalar operator()(const State &s0, const State &s1) const
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