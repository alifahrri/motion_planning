#include <type_traits>
#include <gtest/gtest.h>
#include "tree.hpp"
#include "states.hpp"
#include "trajectory.hpp"

constexpr size_t n = 5; 
constexpr size_t segment = 10;
using state_t = State<double,n>;
using states_t = States<double,n>;
using edge_t = mpl::Trajectory<double,State<double,n>,segment,n>;

TEST(Tree, tree)
{
    mpl::Tree<n,double,states_t,edge_t> tree;
    static_assert(
        std::is_same_v<
            decltype(tree)::kdtree_t,
            KDTree<states_t,n,double,state_t>
        >
    );
    state_t state;
    state << -2.0, -1.0, 0.0, 1.0, 2.0;
    tree.insert(state,-1);
    auto states = tree.states(0);
    static_assert(
        std::is_same_v<
            std::decay_t<decltype(states.at(0))>,
            state_t
        >
    );
    EXPECT_EQ(states.size(), 1);
    EXPECT_EQ(states.at(0), state);
}

TEST(Tree, insert)
{
    mpl::Tree<n,double,states_t,edge_t> tree;
    state_t state;
    {
        state << -2.0, -1.0, 0.0, 1.0, 2.0;
        auto index = tree.insert(state,-1);
        EXPECT_EQ(index, 0);
        EXPECT_EQ(tree.parent[index],-1);
        EXPECT_EQ(tree.kdtree.size(),1);
    }
    {
        state << -2.0, -1.0, 0.0, 1.0, 2.0;
        auto index = tree.insert(state,0,edge_t(state));
        EXPECT_EQ(index, 1);
        EXPECT_EQ(tree.parent[index],0);
        EXPECT_EQ(tree.kdtree.size(),2);
    }
    {
        state << -2.0, -1.0, 0.0, 1.0, 2.0;
        auto index = tree.insert(state,0,edge_t(state));
        EXPECT_EQ(index, 2);
        EXPECT_EQ(tree.parent[index],0);
        EXPECT_EQ(tree.kdtree.size(),3);
    }
    {
        state << -2.0, -1.0, 0.0, 1.0, 2.0;
        auto index = tree.insert(state,1,edge_t(state));
        EXPECT_EQ(index, 3);
        EXPECT_EQ(tree.parent[index],1);
        EXPECT_EQ(tree.kdtree.size(),4);
    }
    {
        state << -2.0, -1.0, 0.0, 1.0, 2.0;
        auto index = tree.insert(state,3,edge_t(state));
        EXPECT_EQ(index, 4);
        EXPECT_EQ(tree.parent[index],3);
        EXPECT_EQ(tree.kdtree.size(),5);
    }
}

TEST(Tree, reset)
{
    mpl::Tree<n,double,states_t,edge_t> tree;
    state_t state;
    for(size_t i=0; i<10; i++) {
        state << -2.0, -1.0, 0.0, 1.0, 2.0;
        tree.insert(state,(int)i-1);
        auto states = tree.states(0);
        EXPECT_EQ(tree.kdtree.size(), i+1);
    }
    tree.reset();
    EXPECT_EQ(tree.kdtree.size(),0);
    EXPECT_EQ(tree.parent.size(),0);
    EXPECT_EQ(tree.trajectories.size(),0);
}