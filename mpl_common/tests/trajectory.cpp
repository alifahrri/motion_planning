#include <gtest/gtest.h>
#include "trajectory.hpp"
#include "states.hpp"
#include "tree.hpp"

TEST(TimeState, intialization)
{
    constexpr size_t n = 5;
    State<double,n> state;
    state << -2.0, -1.0, 0.0, 1.0, 2.0;
    double time{3.3};
    using state_t = State<double,n>;
    using time_state_t = mpl::TimeState<double,state_t>;
    time_state_t time_state(
        std::make_tuple(time,state)
    );
    for(size_t i=0; i<5; i++) {
        EXPECT_EQ(time_state(i),state(i));
    }
    EXPECT_EQ(time_state(5),time);
}

TEST(ConstantSizeTrajectory, constant_size)
{
    constexpr size_t n = 5;
    constexpr size_t segment = 10;
    using state_t = State<double,n>;
    using time_state_t = mpl::TimeState<double,state_t>;
    mpl::Trajectory<time_state_t,segment> trajectory;
    static_assert(trajectory.size()==segment);
    EXPECT_TRUE(true);
}

TEST(DynamicSizeTrajectory, dynamic_size)
{
    constexpr size_t n = 5;
    constexpr size_t segment = -1;
    using state_t = State<double,n>;
    using time_state_t = mpl::TimeState<double,state_t>;
    mpl::Trajectory<time_state_t,segment> trajectory;
    static_assert(std::is_same_v<decltype(trajectory)::allocator_type,std::vector<time_state_t>::allocator_type>);
    EXPECT_TRUE(true);
}

TEST(ConstantSizeTrajectory, element_assign)
{
    constexpr size_t n = 5;
    constexpr size_t segment = 10;
    using state_t = State<double,n>;
    using time_state_t = mpl::TimeState<double,state_t>;
    mpl::Trajectory<time_state_t,segment> trajectory;
    std::vector<state_t> states;
    std::vector<double> times;
    for(size_t i=0; i<segment; i++) {
        state_t state;
        double time{1.0*i};
        state << -.2*i, -.1*i, 0.0*i, .1*i, .2*i;
        states.push_back(state);
        times.push_back(time);
        trajectory[i] = time_state_t(
            std::make_tuple(times.back(), states.back())
        );
    }
    for(size_t i=0; i<segment; i++) {
        EXPECT_EQ(trajectory.time()[i],times.at(i));
    }
    for(size_t i=0; i<segment; i++) {
        EXPECT_EQ(trajectory.path()[i],states.at(i));
    }
}

TEST(ConstantSizeTrajectory, iterator)
{
    constexpr size_t n = 5;
    constexpr size_t segment = 10;
    using state_t = State<double,n>;
    using time_state_t = mpl::TimeState<double,state_t>;
    mpl::Trajectory<time_state_t,segment> trajectory;
    std::vector<state_t> states;
    std::vector<double> times;
    size_t i=0;
    for(auto &trj : trajectory) {
        state_t state;
        double time{1.0*i};
        state << -.2*i, -.1*i, 0.0*i, .1*i, .2*i;
        states.push_back(state);
        times.push_back(time);
        trj = time_state_t(
            std::make_tuple(times.back(), states.back())
        );
        i++;
    }
    for(size_t i=0; i<segment; i++) {
        EXPECT_EQ(trajectory.time()[i],times.at(i));
    }
    for(size_t i=0; i<segment; i++) {
        EXPECT_EQ(trajectory.path()[i],states.at(i));
    }
}

TEST(ConstantSizeTrajectory, const_iterator)
{
    constexpr size_t n = 5;
    constexpr size_t segment = 10;
    using state_t = State<double,n>;
    using time_state_t = mpl::TimeState<double,state_t>;
    mpl::Trajectory<time_state_t,segment> trajectory;
    std::vector<state_t> states;
    std::vector<double> times;
    for(size_t i=0; i<segment; i++) {
        state_t state;
        double time{1.0*i};
        state << -.2*i, -.1*i, 0.0*i, .1*i, .2*i;
        states.push_back(state);
        times.push_back(time);
        trajectory[i] = time_state_t(
            std::make_tuple(times.back(), states.back())
        );
    }
    { /* time test */
        size_t i=0;
        for(const auto &trj : trajectory) {
            EXPECT_EQ(trj.time(),times.at(i++));
        }
    }
    { /* time test */
        size_t i=0;
        for(const auto &trj : trajectory) {
            EXPECT_EQ(trj.state(),states.at(i++));
        }
    }
}