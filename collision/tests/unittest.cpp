#include <gtest/gtest.h>
#include "elements.hpp"
#include "collision.hpp"
#include "environment.hpp"

struct pt_0t {
  double xv=1.0;
  double yv=2.0;
  auto& x() const { return xv; }
  auto& y() const { return yv; }
  auto& x() { return xv; }
  auto& y() { return yv; }
  inline
  bool operator==(const pt_0t &rhs) const {
    return (x()==rhs.x()) && (y()==rhs.y());
  }
  inline
  bool operator!=(const pt_0t &rhs) const {
    return !(*this == rhs);
  }
};

struct pt_1t {
  double x=1.0;
  double y=2.0;
  inline
  bool operator==(const pt_1t &rhs) const {
    return (x==rhs.x) && (y==rhs.y);
  }
  inline
  bool operator!=(const pt_1t &rhs) const {
    return !(*this == rhs);
  }
};

struct ln_0t {
  pt_0t pt0;
  pt_0t pt1;
  auto& p0() const { return pt0; }
  auto& p1() const { return pt1; }
  auto& p0() { return pt0; }
  auto& p1() { return pt1; }
  inline 
  bool operator == (const ln_0t &rhs) const {
    return (pt0 == rhs.pt0) && (pt1 == rhs.pt1);
  }
  inline 
  bool operator != (const ln_0t &rhs) const {
    return !(*this == rhs);
  }
};

struct ln_1t {
  pt_1t p0;
  pt_1t p1;
  inline 
  bool operator == (const ln_1t &rhs) const {
    return (p0 == rhs.p0) && (p1 == rhs.p1);
  }
  inline 
  bool operator != (const ln_1t &rhs) const {
    return !(*this == rhs);
  }
};

struct poly_0t {
  std::vector<ln_0t> lines;
};

struct poly_1t {
  std::vector<ln_1t> line_vector;
  auto& lines() const { return line_vector; }
  auto& lines() { return line_vector; }
};

struct circle_t {
  double x = 0.0;
  double y = 0.0;
  double r = 1.0;
  inline
  bool operator == (const circle_t &rhs) const {
    return x==rhs.x && y==rhs.y && r==rhs.r;
  }
  inline
  bool operator != (const circle_t &rhs) const {
    return !(*this == rhs);
  }
};

namespace test_data {
  ln_0t l0;
  ln_1t l1;
  ln_1t l2;
  pt_0t p;
  pt_1t p1;
  std::vector<ln_0t> lines0;
  std::vector<ln_1t> lines1;
  std::vector<ln_1t> lines2;
  poly_0t poly0;
  poly_1t poly1;
  poly_1t poly2;
  pt_0t pt0;
  pt_1t pt1;
  circle_t c0;
  circle_t c1;
  std::vector<poly_0t> poly0_vec;
  std::vector<poly_1t> poly1_vec;

  void init() {
    l0.pt0.xv = 1.0; l0.pt0.yv = 1.0;
    l0.pt1.xv = -1.0; l0.pt1.yv = -1.0;
    l1.p0.x = 1.0; l1.p0.y = -1.0;
    l1.p1.x = -1.0; l1.p1.y = 1.0;
    l2.p0.x = -1.0; l2.p0.y = 0.0;
    l2.p1.x = -2.0; l2.p1.y = 1.0;

    pt0.xv = 0.0;
    pt0.yv = 0.0;
    pt1.x = 3.0;
    pt1.y = 0.0;

    c1.r = 0.25;
    c1.x = 3.0;

    {
      ln_0t l;
      l.pt0.xv = -1.; l.pt0.yv = -1.;
      l.pt1.xv = -1.; l.pt1.yv = 1.;
      lines0.push_back(l);
      l.pt0.xv = -1.; l.pt0.yv = 1.;
      l.pt1.xv = 1.; l.pt1.yv = 1.;
      lines0.push_back(l);
      l.pt0.xv = 1.; l.pt0.yv = 1.;
      l.pt1.xv = 1.; l.pt1.yv = -1.;
      lines0.push_back(l);
      l.pt0.xv = 1.; l.pt0.yv = -1.;
      l.pt1.xv = -1.; l.pt1.yv = -1.;
      lines0.push_back(l);
      for(const auto &line : lines0) {
        poly0.lines.push_back(line);
      }
    }

    {
      ln_1t l;
      l.p0.x = 0.; l.p0.y = 0.;
      l.p1.x = 0.; l.p1.y = 2.;
      lines1.push_back(l);
      l.p0.x = 0.; l.p0.y = 2.;
      l.p1.x = 2.; l.p1.y = 2.;
      lines1.push_back(l);
      l.p0.x = 2.; l.p0.y = 2.;
      l.p1.x = 2.; l.p1.y = 0.;
      lines1.push_back(l);
      l.p0.x = 2.; l.p0.y = 0.;
      l.p1.x = 0.; l.p1.y = 0.;
      lines1.push_back(l);
      for(const auto &line : lines1) {
        poly1.line_vector.push_back(line);
      }
    }

    {
      ln_1t l;
      l.p0.x = 3.; l.p0.y = 0.;
      l.p1.x = 3.; l.p1.y = 2.;
      lines2.push_back(l);
      l.p0.x = 3.; l.p0.y = 2.;
      l.p1.x = 5.; l.p1.y = 2.;
      lines2.push_back(l);
      l.p0.x = 5.; l.p0.y = 2.;
      l.p1.x = 5.; l.p1.y = 0.;
      lines2.push_back(l);
      l.p0.x = 5.; l.p0.y = 0.;
      l.p1.x = 3.; l.p1.y = 0.;
      lines2.push_back(l);
      for(const auto &line : lines2) {
        poly2.line_vector.push_back(line);
      }
    }

  // 
    for(size_t i=0; i<4; i++) {
      poly0_vec.push_back(poly0);
      poly1_vec.push_back(poly1);
      auto p0 = poly0_vec.at(i);
      auto p1 = poly1_vec.at(i);
    }
  //
  }

}

using namespace test_data;

TEST(elements, points) {
  using elements::x;
  using elements::y;
  EXPECT_TRUE((x(p)==p.x()) && (x(p1)==p1.x) && (y(p)==p.y()) && (y(p1)==p1.y));
}

TEST(elements, lines) {
  using elements::p0;
  using elements::p1;
  EXPECT_TRUE((p0(l0)==l0.p0()) && (p1(l0)==l0.p1()) && (p0(l1)==l1.p0) && (p1(l1)==l1.p1));
}

TEST(collision, line_line_collide) {
  EXPECT_TRUE(collision::line_line_collision(l0,l1));
}

TEST(collision, line_line_collision_free) {
  EXPECT_FALSE(collision::line_line_collision(l0,l2));
}

TEST(elements, poly) {
  auto ok = true;
  for(size_t i=0; i<lines1.size(); i++) {
    ok = (elements::line(poly0, i) != lines0.at(i) ? false : ok);
    ok = (elements::line(poly1, i) != lines1.at(i) ? false : ok);
  }
  EXPECT_TRUE(ok);
}

TEST(collision, point_poly_collide) {
  EXPECT_TRUE(collision::point_inside_poly(pt0, poly0));
}

TEST(collision, point_poly_collision_free) {
  EXPECT_FALSE(collision::point_inside_poly(pt1, poly0));
}

TEST(collision, poly_poly_collide) {
  EXPECT_TRUE(collision::poly_poly_collision(poly0, poly1));
}

TEST(collision, poly_poly_collision_free) {
  EXPECT_FALSE(collision::poly_poly_collision(poly0, poly2));
}

TEST(collision, circle_poly_collide) {
  EXPECT_TRUE(collision::circle_poly_collision(c0, poly0));
}

TEST(collision, circle_poly_collision_free) {
  EXPECT_FALSE(collision::circle_poly_collision(c1, poly0));
}

TEST(environment, add_polygons) {
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

TEST(environment, add_circles) {
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

TEST(environment, polygon_env_collide) {
  auto env = environment::Environment<double>();
  {
    env.add_polygons(poly0);
    env.add_polygons(poly1);
  }
  EXPECT_TRUE(env.collide(poly1));
}

TEST(environment, polygon_env_collision_free) {
  auto env = environment::Environment<double>();
  {
    env.add_polygons(poly0);
    env.add_polygons(poly1);
  }
  EXPECT_FALSE(env.collide(poly2));
}

// TEST(environment, circle_env_collide) {
//   auto env = environment::Environment<double>();
//   {
//     env.add_polygons(poly0);
//     env.add_polygons(poly1);
//   }
//   EXPECT_TRUE(env.collide(c0));
// }

// TEST(environment, circle_env_collision_free) {
//   auto env = environment::Environment<double>();
//   {
//     env.add_polygons(poly0);
//     env.add_polygons(poly1);
//   }
//   EXPECT_FALSE(env.collide(c1));
// }

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  // Models::init_integrator2d();
  test_data::init();
  return RUN_ALL_TESTS();
}

