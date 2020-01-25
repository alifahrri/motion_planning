#ifndef NEIGHBOR_HPP
#define HEIGHBOR_HPP

#include <ratio>
#include <cmath>

namespace mpl {
    namespace helper {
        /* define compile-time multiplication */
        template <size_t w, size_t ... ws>
        struct Multiplication {
            enum { result = w * Multiplication<ws...>::result };
        };
        template <size_t w>
        struct Multiplication<w> {
            enum { result = w };
        };
    } // namespace helper
    
    /* neighbor radius with compile-time constants */
    template<typename Index, typename Scalar, size_t denom=0, size_t ... Workspaces>
    struct NeighborRadius {
        constexpr NeighborRadius(Scalar scale = Scalar{1})
            : scale(scale) {}
        constexpr static size_t dim = sizeof...(Workspaces);
        const Scalar ws[dim] = {
            (Workspaces/denom)...
        };
        constexpr static Scalar s = std::pow(2,dim) * (1+1/Scalar{dim}) * helper::Multiplication<Workspaces...>::result / denom;
        const Scalar scale;
        constexpr Scalar operator()(Index i) const {
            return scale*s*std::log(i+1)/(i+1);
        }
    };
    template <typename Index, typename Scalar>
    struct NeighborRadius<Index,Scalar,0> {
        template <typename Iterable>
        NeighborRadius(const Iterable &workspaces, Scalar scale = Scalar{1}) 
            : scale(scale)
        {
            auto v = Scalar{1};
            for(const auto &w : workspaces)
                v *= w;
            s = std::pow(2,std::size(workspaces)) * (1+1/Scalar{std::size(workspaces)}) * v;
        }
        Scalar scale;
        Scalar s;
        Scalar operator()(Index i) const {
            return scale*s*std::log(i+1)/(i+1);
        }
    };
} // namespace mpl

#endif // HEIGHBOR_HPP