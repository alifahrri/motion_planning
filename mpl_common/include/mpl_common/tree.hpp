#ifndef TREE_HPP
#define TREE_HPP

#include <cstdio>
#include <vector>
#include "kdtree.hpp"

namespace mpl {
    template <size_t dim, typename Scalar, typename States, typename Edge>
    struct Tree {
        using State = typename States::state_type;
        using Index = int;
        using IndexList = std::vector<Index>;
        using StateList = std::vector<State>;
        using EdgeList = std::vector<Edge>;
        using kdtree_t = KDTree<States,dim,Scalar,State>;

        Tree() {}

        constexpr
        auto nearest(const State &s, const Scalar &radius)
        {
            IndexList near;
            auto vp = kdtree.nearest(s,radius);
            for(const auto & p : vp) 
                near.push_back(Index(p.first));
            return near;
        }

        constexpr
        auto states(const IndexList &indexes)
        {
            StateList ret;
            for(const auto &i : indexes)
                ret.push_back(kdtree(i));
            return ret;
        }

        constexpr
        auto states(const Index index)
        {
            StateList ret;
            auto i = index;
            while(i>=0) {
                ret.insert(ret.begin(), kdtree(i));
                i = parent[i];
            }
            return ret;
        }

        constexpr 
        auto insert(const State &s, const Index idx)
        {
            Index id = kdtree.size();
            auto e = Edge(s);
            kdtree.addPoint(s);
            setParent(id,idx);
            setEdge(id,e);
            return id;
        }

        constexpr
        auto insert(const State &s, const Index idx, const Edge &e)
        {
            Index id = kdtree.size();
            kdtree.addPoint(s);
            parent.push_back(idx);
            trajectories.push_back(e);
            return id;
        }

        void reset()
        {
            last_checked_idx = -1;
            kdtree.clear();
            parent.clear();
            trajectories.clear();
        }

        constexpr
        void setParent(const Index node, const Index p)
        {
            if(Index(parent.size()) < node+1)
                parent.push_back(p);
            else parent.at(node) = p;
        }

        constexpr
        void setEdge(const Index n, const Edge &e)
        {
            if(Index(trajectories.size()) < n+1)
                trajectories.push_back(e);
            else trajectories.at(n) = e;
        }

        constexpr
        auto& operator()(const Index i)
        {
            last_checked_idx = i;
            return kdtree(i);
        }
        
        constexpr
        auto time_offset() -> Scalar  {
            if(last_checked_idx) {
                // return std::get<0>(trajectories.at(last_checked_idx).back());
                return trajectories.at(last_checked_idx).back().time();
            }
            return 0;
        }

        Edge get_trajectory(Index idx) 
        {
            auto i = idx;
            Edge sol;
            while(i>0) {
                sol.insert(sol.begin(), trajectories[i]);
                i = parent[i];
            }
            return sol;
        }

        Index last_checked_idx = -1;
        IndexList parent;
        kdtree_t kdtree;
        EdgeList trajectories;
    };
} // namespace mpl

#endif // TREE_HPP