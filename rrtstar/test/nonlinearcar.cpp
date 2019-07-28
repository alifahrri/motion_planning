#include <gtest/gtest.h>
#include "models/dynamics/nonlinearcar.hpp"
#include "mpl_common/trajectory.hpp"
#include "mpl_common/states.hpp"
#include "mpl_common/cost.hpp"
#include "rrtstar/nonlinearcar.hpp"

TEST(NonlinearCarState, state_assign)
{
    using state_t = State<double,Models::n>;
    state_t xi, xf;
    xf << 1,1,1,1,1;
    xi << 0,0,0,0,0;
    for(size_t i=0; i<5; i++) {
        EXPECT_NEAR(xf(i),1,1e-15);
        EXPECT_NEAR(xi(i),0,1e-15);
    }
}

TEST(NonlinearCarCost, cost_no_inf_nan)
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

TEST(NonlinearCarCost, cost_non_negative)
{
    Models::NonlinearCar car;
    using state_t = State<double,Models::n>;
    using cost_t = mpl::Cost<double,state_t,decltype(car.solver)>;
    cost_t cost(car.solver);
    /* linearization */
    state_t xi, xf; 
    xf << 1,1,1,1,1;
    xi << 0,0,0,0,0;
    car.linearize(xf);
    auto c = cost(xi,xf);
    EXPECT_GT(c,0.0);
}

TEST(NonlinearCarCost, cost_equal_to_solver_cost)
{
    Models::NonlinearCar car;
    using state_t = State<double,Models::n>;
    using cost_t = mpl::Cost<double,state_t,decltype(car.solver)>;
    /* linearization */
    state_t xi, xf; 
    xf << 1,1,1,1,1;
    xi << 0,0,0,0,0;
    car.linearize(xf);
    cost_t cost(car.solver);
    auto c = cost(xi,xf);
    EXPECT_EQ(c,std::get<1>(car.solver.cost(xi,xf)));
}

TEST(NonlinearCarSampler, random_sample_no_inf_nan)
{
    auto sampler = Kinodynamic::Car::CarSampler();
    auto s0 = sampler();
    for(const auto &s : s0)
        EXPECT_FALSE(std::isnan(s) || std::isinf(s));
}

TEST(NonlinearCarSampler, random_sample_bounded)
{
    auto sampler = Kinodynamic::Car::CarSampler();
    std::array<double,5> lower_bound{-10,-10,-M_PI,-1,-1};
    std::array<double,5> upper_bound{10,10,M_PI,1,1};
    sampler.set_bound(lower_bound,upper_bound);
    auto s = sampler();
    for(size_t i=0; i<lower_bound.size(); i++) {
        EXPECT_GE(s[i],lower_bound[i]);
        EXPECT_GE(upper_bound[i],s[i]);
    }
}

/*
TEST(nonlinearcar_rrt, environment_collision_free)
{
    auto env = Kinodynamic::Car::Environment();
    auto sampler = Kinodynamic::Car::CarSampler();
    auto state = sampler();
    EXPECT_FALSE(env.collide(state));
}
*/


TEST(NonlinearCarConnector, non_empty_solver_trajectory)
{
    using state_t = State<double,Models::n>;
    auto sampler = Kinodynamic::Car::CarSampler();
    auto car = Models::NonlinearCar();
    auto &solver = car.solver;
    auto tree = Kinodynamic::Car::CarTree();
    auto connector = Kinodynamic::Car::CarConnector(solver,tree);
    state_t xi, xf; 
    xf << 1,1,1,1,1;
    xi << 0,0,0,0,0;
    car.linearize(xf);
    auto trajectory = connector.template operator()<decltype(solver.solve(xi,xf))>(xi,xf);
    EXPECT_TRUE(trajectory.size());
    for(const auto &t : trajectory)
        for(size_t i=0; i<Models::n; i++)
            EXPECT_FALSE(std::isinf(std::get<1>(t)(i)) || std::isnan(std::get<1>(t)(i))) << std::get<1>(t)(i);
}

TEST(NonlinearCarConnector, non_empty_solver_dynamic_trajectory)
{
    using state_t = State<double,Models::n>;
    auto sampler = Kinodynamic::Car::CarSampler();
    auto car = Models::NonlinearCar();
    auto &solver = car.solver;
    auto tree = Kinodynamic::Car::CarTree();
    auto connector = Kinodynamic::Car::CarConnector(solver,tree);
    state_t xi, xf; 
    xf << 1.,1.,1.,1.,1.;
    xi << 0.,0.,0.,0.,0.;
    car.linearize(xf);
    size_t n = 99;
    auto trajectory = connector.template operator()<decltype(solver.solve(xi,xf,n))>(xi,xf,n);
    EXPECT_TRUE(trajectory.size());
    for(const auto &t : trajectory)
        for(size_t i=0; i<Models::n; i++)
            EXPECT_FALSE(std::isinf(std::get<1>(t)(i)) || std::isnan(std::get<1>(t)(i)) ) << std::get<1>(t)(i);
}

TEST(NonlinearCarConnector, non_empty_trajectory)
{
    using state_t = State<double,Models::n>;
    auto sampler = Kinodynamic::Car::CarSampler();
    auto car = Models::NonlinearCar();
    auto &solver = car.solver;
    auto tree = Kinodynamic::Car::CarTree();
    auto connector = Kinodynamic::Car::CarConnector(solver,tree);
    state_t xi, xf; 
    xf << 1,1,1,1,1;
    xi << 0,0,0,0,0;
    car.linearize(xf);
    auto trajectory = connector(xi,xf);
    EXPECT_TRUE(trajectory.size());
    for(const auto &t : trajectory)
        for(size_t i=0; i<Models::n; i++)
            EXPECT_FALSE(std::isinf(t.state()(i)) || std::isnan(t.state()(i)));
}

TEST(NonlinearCarConnector, init_and_final_state)
{
    auto sampler = Kinodynamic::Car::CarSampler();
    auto car = Models::NonlinearCar();
    auto &solver = car.solver;
    auto tree = Kinodynamic::Car::CarTree();
    auto connector = Kinodynamic::Car::CarConnector(solver,tree);
    std::array<std::decay_t<decltype(sampler())>,10> states = {
        sampler(), sampler(), sampler(), sampler(), sampler(),
        sampler(), sampler(), sampler(), sampler(), sampler()
    };
    // for(auto &s : states) s = sampler();
    constexpr auto zero = 1e-5;
    for(size_t i=1; i<states.size(); i++) {
        Kinodynamic::Car::ModelState xf, xi;
        xf << states[i][0], states[i][1], states[i][2], states[i][3], states[i][4];
        xi << states[i-1][0], states[i-1][1], states[i-1][2], states[i-1][3], states[i-1][4];
        car.linearize(xf);
        auto trajectory = connector(xi, xf);
        for(size_t k=0; k<Kinodynamic::Car::N; k++) {
            EXPECT_NEAR(trajectory.front().state()(k),states[i-1][k],zero);
            EXPECT_NEAR(trajectory.back().state()(k),states[i][k],zero);
        }
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
