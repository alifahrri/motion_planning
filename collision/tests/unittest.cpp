#include <gtest/gtest.h>
#include "test_data.h"

using namespace test_data;

TEST(elements, points)
{
	using elements::x;
	using elements::y;
	EXPECT_TRUE((x(p)==p.x()) && (x(p1)==p1.x) && (y(p)==p.y()) && (y(p1)==p1.y));
}

TEST(elements, lines)
{
	using elements::p0;
	using elements::p1;
	EXPECT_TRUE((p0(l0)==l0.p0()) && (p1(l0)==l0.p1()) && (p0(l1)==l1.p0) && (p1(l1)==l1.p1));
}

TEST(collision, line_line_collide)
{
	EXPECT_TRUE(collision::line_line_collision(l0,l1));
}

TEST(collision, line_line_collision_free)
{
	EXPECT_FALSE(collision::line_line_collision(l0,l2));
}

TEST(elements, poly)
{
	auto ok = true;
	for(size_t i=0; i<lines1.size(); i++) {
		ok = (elements::line(poly0, i) != lines0.at(i) ? false : ok);
		ok = (elements::line(poly1, i) != lines1.at(i) ? false : ok);
	}
	EXPECT_TRUE(ok);
}

TEST(collision, point_poly_collide)
{
	EXPECT_TRUE(collision::point_inside_poly(pt0, poly0));
}

TEST(collision, point_poly_collision_free)
{
	EXPECT_FALSE(collision::point_inside_poly(pt1, poly0));
}

TEST(collision, poly_poly_collide)
{
	EXPECT_TRUE(collision::poly_poly_collision(poly0, poly1));
}

TEST(collision, poly_poly_collision_free)
{
	EXPECT_FALSE(collision::poly_poly_collision(poly0, poly2));
}

TEST(collision, circle_poly_collide)
{
	EXPECT_TRUE(collision::circle_poly_collision(c0, poly0));
}

TEST(collision, circle_poly_collision_free)
{
	EXPECT_FALSE(collision::circle_poly_collision(c1, poly0));
}

TEST(environment, add_polygons)
{
	auto ok = true;
	auto env = environment::Environment<double>();

	using elements::x;
	using elements::y;
	using elements::p0;
	using elements::p1;

	auto i = 0;
	auto check0 = [&](const auto &line) {
		ok = (x(p0(line)) != x(p0(elements::line(poly0,i))) ? false : ok);
		ok = (y(p0(line)) != y(p0(elements::line(poly0,i))) ? false : ok);
		ok = (x(p1(line)) != x(p1(elements::line(poly0,i))) ? false : ok);
		ok = (y(p1(line)) != y(p1(elements::line(poly0,i))) ? false : ok);
		i++;
	};
	auto check1 = [&](const auto &line) {
		ok = (x(p0(line)) != x(p0(elements::line(poly1,i))) ? false : ok);
		ok = (y(p0(line)) != y(p0(elements::line(poly1,i))) ? false : ok);
		ok = (x(p1(line)) != x(p1(elements::line(poly1,i))) ? false : ok);
		ok = (y(p1(line)) != y(p1(elements::line(poly1,i))) ? false : ok);
		i++;
	};

	{
		env.add_polygons(poly0);
		i = 0;
		elements::poly_iter(env.polygons.back(),check0);
	}

	{
		env.add_polygons(poly1);
		i = 0;
		elements::poly_iter(env.polygons.back(),check1);
	}

	{
		env.add_polygons(poly0_vec);
		for(size_t k=0; k<4; k++) {
			i = 0;
			elements::poly_iter(env.polygons.at(env.polygons.size()-4+k),check0);
		}
	}

	{
		env.add_polygons(poly1_vec);
		for(size_t k=0; k<4; k++) {
			i = 0;
			elements::poly_iter(env.polygons.at(env.polygons.size()-4+k),check1);
		}
	}

	EXPECT_TRUE(ok);
}

TEST(environment, add_circles)
{
	auto ok = true;
	auto env = environment::Environment<double>();

	using elements::x;
	using elements::y;
	using elements::radius;
	using elements::p0;
	using elements::p1;

	auto check = [&](const auto &lhs, const auto &rhs) {
		return (x(lhs)==x(rhs)) && (y(lhs)==y(rhs)) && (radius(lhs)==radius(rhs));
	};

	{
		env.add_circles(c0);
		ok = check(env.circles.back(),c0) ? ok : false;
	}

	{
		std::vector<decltype(c0)> cvec;
		cvec.push_back(c0);
		cvec.push_back(c1);
		env.add_circles(cvec);
		for(size_t i=1; i<env.circles.size(); i++)
			ok = check(env.circles.at(i),cvec.at(i-1)) ? ok : false;
	}

	EXPECT_TRUE(ok);
}

// TODO : FIX THIS!

//TEST(environment, polygon_env_collide)
//{
//	auto env = environment::Environment<double>();
//	{
//		env.add_polygons(poly0);
//		env.add_polygons(poly1);
//	}
//	// collision between polygon type
//	EXPECT_TRUE(env.collide(poly1));
//}

//TEST(environment, polygon_env_collision_free)
//{
//	auto env = environment::Environment<double>();
//	{
//		env.add_polygons(poly0);
//		env.add_polygons(poly1);
//	}
//	// no collision between polygon
//	EXPECT_FALSE(env.collide(poly2));
//}

 TEST(environment, circle_env_collide) {
   auto env = environment::Environment<double>();
   {
     env.add_polygons(poly0);
     env.add_polygons(poly1);
   }
	 // collision between circle and polygons
   EXPECT_TRUE(env.collide(c0));
 }

 TEST(environment, circle_env_collision_free) {
   auto env = environment::Environment<double>();
   {
     env.add_polygons(poly0);
     env.add_polygons(poly1);
   }
   EXPECT_FALSE(env.collide(c1));
 }

TEST(gridmap, no_collision)
{
	auto& grid = octo_wrapper;
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

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc,argv);
	// Models::init_integrator2d();
	test_data::init();
	return RUN_ALL_TESTS();
}
