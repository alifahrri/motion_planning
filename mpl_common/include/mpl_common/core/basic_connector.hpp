#ifndef BASIC_CONNECTOR_HPP
#define BASIC_CONNECTOR_HPP

namespace mpl {
    namespace core {
        
        /**
         * @brief : crtp base for connector, encapsulate solver
         * @tparam : T
         * @requirements :
         * - T should have member type solver_state_type
         * - T should have member function solver
         * - solver (of T) should have member function solve
         */
        template <typename T>
        struct BaseConnetor 
        {
            template <typename trajectory_t, typename istate_t, typename fstate_t, size_t nsegment = 10>
            trajectory_t operator() (const istate_t &xi, const fstate_t &xf) {
                using solver_state_type = typename T::solver_state_type;
                trajectory_t trajectory;
                auto result = static_cast<T*>(this)->solver.template solve<nsegment-1>(
                    solver_state_type(xi),
                    solver_state_type(xf)
                );
                trajectory = result;
                return trajectory;
            }
            
            template <typename trajectory_t, typename istate_t, typename fstate_t>
            trajectory_t operator() (const istate_t &xi, const fstate_t &xf, size_t nsegment) {
                using solver_state_type = typename T::solver_state_type;
                trajectory_t trajectory;
                auto result = static_cast<T*>(this)->solver.solve(
                    solver_state_type(xi),
                    solver_state_type(xf), 
                    nsegment
                );
                trajectory = result;
                return trajectory;
            }
            
            template <typename istate_t, typename fstate_t>
            auto operator() (const istate_t &xi, const fstate_t &xf) {
                using solver_state_type = typename T::solver_state_type;
                using trajectory_t = typename T::edge_type;
                constexpr int nsegment = 10;
                trajectory_t trajectory;
                auto result = static_cast<T*>(this)->solver.template solve<nsegment-1>(
                    solver_state_type(xi),
                    solver_state_type(xf)
                );
                trajectory = result;
                return trajectory;
            }
        };
    } // namespace core
} // namespace mpl
#endif // BASIC_CONNECTOR_HPP