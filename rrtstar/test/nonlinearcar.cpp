#include <gtest/gtest.h>
#include "models/dynamics/nonlinearcar.hpp"
#include "mpl_common/trajectory.hpp"
#include "mpl_common/states.hpp"
#include "mpl_common/cost.hpp"

TEST(nonlinearcar_rrt, cost_no_inf_nan)
{
    Models::NonlinearCar car;
    using state_t = State<double,Models::n>;
    using cost_t = mpl::Cost<double,state_t,decltype(car.solver)>;
    cost_t cost(car.solver);
    static_assert(
        std::is_same_v<
            decltype(car.solver),
            std::decay_t<decltype(cost.solver)>
        >
    );
    static_assert(
        std::is_same_v<
            std::result_of_t<cost_t(cost_t::state_type,cost_t::state_type)>,
            double
        >
    );
    /* linearization */
    state_t xi, xf; 
    xf << 1,1,1,1,1;
    xi << 0,0,0,0,0;
    car.linearize(xf);
    auto c = cost(xi,xf);
    EXPECT_TRUE(!std::isnan(c));
    EXPECT_TRUE(!std::isinf(c));
}