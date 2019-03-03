#include <gtest/gtest.h>
#include "test_data.h"

using namespace test_data;

TEST(collision, line_line_collide)
{
	TestData t;
	EXPECT_TRUE(collision::line_line_collision(t.l0,t.l1));
}

TEST(collision, line_line_collision_free)
{
	TestData t;
	EXPECT_FALSE(collision::line_line_collision(t.l0,t.l2));
}

TEST(collision, point_poly_collide)
{
	TestData t;
	EXPECT_TRUE(collision::point_inside_poly(t.pt0, t.poly0));
}

TEST(collision, point_poly_collision_free)
{
	TestData t;
	EXPECT_FALSE(collision::point_inside_poly(t.pt1, t.poly0));
}

TEST(collision, poly_poly_collide)
{
	TestData t;
	EXPECT_TRUE(collision::poly_poly_collision(t.poly0, t.poly1));
}

TEST(collision, poly_poly_collision_free)
{
	TestData t;
	EXPECT_FALSE(collision::poly_poly_collision(t.poly0, t.poly2));
}

TEST(collision, circle_poly_collide)
{
	TestData t;
	EXPECT_TRUE(collision::circle_poly_collision(t.c0, t.poly0));
}

TEST(collision, circle_poly_collision_free)
{
	TestData t;
	EXPECT_FALSE(collision::circle_poly_collision(t.c1, t.poly0));
}