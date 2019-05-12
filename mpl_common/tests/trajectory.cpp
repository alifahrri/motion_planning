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
    mpl::TimeState<double,State<double,n>,n> time_state(
        std::make_tuple(time,state)
    );
    for(size_t i=0; i<5; i++) {
        EXPECT_EQ(time_state(i),state(i));
    }
    EXPECT_EQ(time_state(5),time);
}

TEST(Trajectory, path_time)
{
    constexpr size_t n = 5;
    constexpr size_t segment = 10;
    mpl::Trajectory<double,State<double,n>,segment,n> trajectory;
    std::vector<State<double,n>> states;
    std::vector<double> times;
    for(size_t i=0; i<=segment; i++) {
        State<double,n> state;
        double time{1.0*i};
        state << -.2*i, -.1*i, 0.0*i, .1*i, .2*i;
        states.push_back(state);
        times.push_back(time);
        trajectory[i] = mpl::TimeState<double,State<double,n>,n>(
            std::make_tuple(times.back(), states.back())
        );
    }
    for(size_t i=0; i<=segment; i++) {
        EXPECT_EQ(trajectory.time()[i],times.at(i));
    }
    for(size_t i=0; i<=segment; i++) {
        EXPECT_EQ(trajectory.path()[i],states.at(i));
    }
}