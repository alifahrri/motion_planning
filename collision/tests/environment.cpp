#include <gtest/gtest.h>
#include "test_data.h"

using namespace test_data;

TEST(environment, add_polygons)
{
	TestData t;
	auto ok = true;
	auto env = environment::Environment<double>();

	using elements::x;
	using elements::y;
	using elements::p0;
	using elements::p1;

	auto i = 0;
	auto check0 = [&](const auto &line) {
		ok = (x(p0(line)) != x(p0(elements::line(t.poly0,i))) ? false : ok);
		ok = (y(p0(line)) != y(p0(elements::line(t.poly0,i))) ? false : ok);
		ok = (x(p1(line)) != x(p1(elements::line(t.poly0,i))) ? false : ok);
		ok = (y(p1(line)) != y(p1(elements::line(t.poly0,i))) ? false : ok);
		i++;
	};
	auto check1 = [&](const auto &line) {
		ok = (x(p0(line)) != x(p0(elements::line(t.poly1,i))) ? false : ok);
		ok = (y(p0(line)) != y(p0(elements::line(t.poly1,i))) ? false : ok);
		ok = (x(p1(line)) != x(p1(elements::line(t.poly1,i))) ? false : ok);
		ok = (y(p1(line)) != y(p1(elements::line(t.poly1,i))) ? false : ok);
		i++;
	};

	{
		env.add_polygons(t.poly0);
		i = 0;
		elements::poly_iter(env.polygons.back(),check0);
	}

	{
		env.add_polygons(t.poly1);
		i = 0;
		elements::poly_iter(env.polygons.back(),check1);
	}

	{
		env.add_polygons(t.poly0_vec);
		for(size_t k=0; k<4; k++) {
			i = 0;
			elements::poly_iter(env.polygons.at(env.polygons.size()-4+k),check0);
		}
	}

	{
		env.add_polygons(t.poly1_vec);
		for(size_t k=0; k<4; k++) {
			i = 0;
			elements::poly_iter(env.polygons.at(env.polygons.size()-4+k),check1);
		}
	}

	EXPECT_TRUE(ok);
}

TEST(environment, add_circles)
{
	TestData t;
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
		env.add_circles(t.c0);
		ok = check(env.circles.back(),t.c0) ? ok : false;
	}

	{
		std::vector<decltype(t.c0)> cvec;
		cvec.push_back(t.c0);
		cvec.push_back(t.c1);
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

TEST(environment, circle_env_collide)
{
	TestData t;
	auto env = environment::Environment<double>();
	{
		env.add_polygons(t.poly0);
		env.add_polygons(t.poly1);
	}
	// collision between circle and polygons
	EXPECT_TRUE(env.collide(t.c0));
}

TEST(environment, circle_env_collision_free)
{
	TestData t;
	auto env = environment::Environment<double>();
	{
		env.add_polygons(t.poly0);
		env.add_polygons(t.poly1);
	}
	EXPECT_FALSE(env.collide(t.c1));
}
