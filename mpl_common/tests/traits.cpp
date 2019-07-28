#include "traits.hpp"
#include <gtest/gtest.h>
#include <array>
#include <vector>

TEST(traits, is_dynamic_size_container)
{
    std::array<double,3> a{ 1., 2., 3. };
    std::vector<double> v{ 1., 2., 3. };
    
    static_assert( mpl::traits::is_dynamic_size_container<decltype(v)>::value);
    static_assert(!mpl::traits::is_dynamic_size_container<decltype(a)>::value);
}