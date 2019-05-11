#include <gtest/gtest.h>
#include "test_data.h"

using namespace test_data;

TEST(gridmap, no_collision)
{
	TestData t;
	auto& grid = t.octo_wrapper;
	double origin[3] = {0.0,0.0,0.0};
	std::vector<std::array<double,3>> test_point;
	test_point.push_back({-0.4,-0.4,-0.4});
	test_point.push_back({1.0,-1.0,-1.0});
	test_point.push_back({-1.0,1.0,-1.0});
	test_point.push_back({1.0,1.0,-1.0});
	test_point.push_back({-1.0,-1.0,1.0});
	test_point.push_back({1.0,-1.0,1.0});
	test_point.push_back({-1.0,1.0,1.0});
	test_point.push_back({1.0,1.0,1.0});
	decltype(grid.collide(origin, test_point.front())) collision{0};
	std::stringstream ss;
	for(const auto& endpoint : test_point) {
		auto collide = grid.collide(origin, endpoint);
		for(const auto& p : endpoint)
			ss << p << ",";
		ss << "->" << std::boolalpha << collide << "; ";
		if(collide) collision = true;
	}
//	double endpoint[3] = {-1.0,-1.0,-1.0};
//	collision = grid.collide(origin, endpoint);
//	ss << std::boolalpha << collision;
	EXPECT_FALSE(collision) << ss.str();
}