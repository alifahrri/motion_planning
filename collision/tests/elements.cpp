#include <gtest/gtest.h>
#include <test_data.h>

using namespace test_data;

TEST(elements, points)
{
	TestData t;
	using elements::x;
	using elements::y;
	EXPECT_TRUE((x(t.p)==t.p.x()) && (x(t.p1)==t.p1.x) && (y(t.p)==t.p.y()) && (y(t.p1)==t.p1.y));
}

TEST(elements, lines)
{
	TestData t;
	using elements::p0;
	using elements::p1;
	EXPECT_TRUE((p0(t.l0)==t.l0.p0()) && (p1(t.l0)==t.l0.p1()) && (p0(t.l1)==t.l1.p0) && (p1(t.l1)==t.l1.p1));
}

TEST(elements, poly)
{
	TestData t;
	auto ok = true;
	for(size_t i=0; i<t.lines1.size(); i++) {
		ok = (elements::line(t.poly0, i) != t.lines0.at(i) ? false : ok);
		ok = (elements::line(t.poly1, i) != t.lines1.at(i) ? false : ok);
	}
	EXPECT_TRUE(ok);
}