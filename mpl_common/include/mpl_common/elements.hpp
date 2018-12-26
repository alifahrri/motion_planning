#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

#include "choice.hpp"

namespace elements {

template<size_t idx>
inline
const auto& element(choice<1>, const auto &p) {
  return p(idx);
}

template<size_t idx>
inline
const auto& element(choice<0>, const auto &p) {
  return p[idx];
}

inline
const auto& element(choice<1>, const auto &p, size_t idx) {
  return p(idx);
}

inline
const auto& element(choice<0>, const auto &p, size_t idx) {
  return p[idx];
}

template<size_t idx>
inline
auto& element(choice<1>, auto &p) {
  return p(idx);
}

template<size_t idx>
inline
auto& element(choice<0>, auto &p) {
  return p[idx];
}

inline
auto& element(choice<1>, auto &p, size_t idx) {
  return p(idx);
}

inline
auto& element(choice<0>, auto &p, size_t idx) {
  return p[idx];
}

/* generic getter functino for x and y elements
 *
 */

template<size_t idx>
inline
auto x(choice<2>, const auto &p) -> decltype(p.x()) {
  return p.x();
}

template<size_t idx>
inline
auto x(choice<1>, const auto &p) -> decltype(p.x) {
  return p.x;
}

template<size_t idx>
inline
auto x(choice<0>, const auto &p) {
  return element<idx>(choice<1>{},p);
}

template<size_t idx=0>
inline
auto x(const auto &p) {
  return x<idx>(choice<2>{},p);
}

template<size_t idx>
inline
auto y(choice<2>, const auto &p) -> decltype(p.y()) {
  return p.y();
}

template<size_t idx>
inline
auto y(choice<1>, const auto &p) -> decltype(p.y) {
  return p.y;
}

template<size_t idx>
inline
auto y(choice<0>, const auto &p) {
  return element<idx>(choice<1>{},p);
}

template<size_t idx=1>
inline
auto y(const auto &p) {
  return y<idx>(choice<2>{},p);
}

/* get reference of the elements
 */
template<size_t idx>
inline
auto x(choice<2>, auto &p) -> decltype((p.x())) {
  return p.x();
}

template<size_t idx>
inline
auto x(choice<1>, auto &p) -> decltype((p.x)) {
  return p.x;
}

template<size_t idx>
inline
auto x(choice<0>, auto &p) {
  return element<idx>(choice<1>{},p);
}

template<size_t idx=0>
inline
auto x(auto &p) {
  return x<idx>(choice<2>{},p);
}

template<size_t idx>
inline
auto y(choice<2>, auto &p) -> decltype((p.y())) {
  return p.y();
}

template<size_t idx>
inline
auto y(choice<1>, auto &p) -> decltype((p.y)) {
  return p.y;
}

template<size_t idx>
inline
auto y(choice<0>, auto &p) {
  return element<idx>(choice<1>{},p);
}

template<size_t idx=1>
inline
auto y(auto &p) {
  return y<idx>(choice<2>{},p);
}

/* generic getter function for circles
 */
template <size_t r_idx=2>
inline
auto radius(choice<5>, const auto &circle) -> decltype(circle.r()) {
  return circle.r();
}

template <size_t r_idx=2>
inline
auto radius(choice<4>, const auto &circle) -> decltype(circle.r) {
  return circle.r;
}

template <size_t r_idx=2>
inline
auto radius(choice<3>, const auto &circle) -> decltype(circle.radius()) {
  return circle.radius();
}

template <size_t r_idx=2>
inline
auto radius(choice<2>, const auto &circle) -> decltype(circle.radius) {
  return circle.radius;
}

template <size_t r_idx=2>
inline
auto radius(choice<1>, const auto &circle) -> decltype(circle.radius()) {
  return circle.radius();
}

template <size_t r_idx=2>
inline
auto radius(choice<0>, const auto &circle) {
  return element<r_idx>(choice<1>{}, circle);
}

template <size_t r_idx=2>
inline
auto radius(const auto &circle) {
  return radius<r_idx>(choice<5>{}, circle);
}

/* generic getter functino for points of line
 *
 */
template<size_t idx>
inline
auto p0(choice<2>, const auto &l) -> decltype(l.p0()) {
  return l.p0();
}

template<size_t idx>
inline
auto p0(choice<1>, const auto &l) -> decltype(l.p0) {
  return l.p0;
}

template<size_t idx>
inline
auto p0(choice<0>, const auto &l) {
  return element<idx>(choice<1>{}, l);
}

template<size_t idx=0>
inline
auto p0(const auto &l) {
  return p0<idx>(choice<2>{},l);
}

template<size_t idx>
inline
auto p1(choice<2>, const auto l) -> decltype(l.p1()) {
  return l.p1();
}

template<size_t idx>
inline
auto p1(choice<1>, const auto &l) -> decltype(l.p1) {
  return l.p1;
}

template<size_t idx>
inline
auto p1(choice<0>, const auto &l) {
  return element<idx>(choice<1>{}, l);
}

template<size_t idx=0>
inline
auto p1(const auto &l) {
  return p1<idx>(choice<2>{},l);
}

/* Generic getter function for line from polygon
 */

inline
auto line(choice<2>, const auto &poly, size_t i) -> decltype(element(poly.lines, i)) {
  return element(choice<1>{}, poly.lines, i);
}

inline
auto line(choice<1>, const auto &poly, size_t i) -> decltype(element(poly.lines(), i)) {
  return element(choice<1>{}, poly.lines(), i);
}

inline
auto line(choice<0>, const auto &poly, size_t i) {
  return element(choice<1>{}, poly, i);
}

inline
auto line(const auto &poly, size_t i) {
  return line(choice<2>{}, poly, i);
}

/* Generic iterator for polygon
 */

inline
auto poly_iter(choice<3>, const auto &poly, auto &fn) -> decltype(poly.begin(), poly.end(), void()) {
  for(const auto& p : poly) {
    fn(p);
  }
}

inline
auto poly_iter(choice<2>, const auto &poly, auto &fn) -> decltype(poly.size(), poly.at(0), void()) {
  for(const auto& p : poly) {
    fn(p);
  }
}

inline
auto poly_iter(choice<1>, const auto &poly, auto &fn) -> decltype(poly.lines, void()) {
  for(const auto& p : poly.lines) {
    fn(p);
  }
}

inline
auto poly_iter(choice<0>, const auto &poly, auto &fn) -> decltype(poly.lines(), void()) {
  for(const auto& p : poly.lines()) {
    fn(p);
  }
}

inline
auto poly_iter(const auto &poly, auto &fn) {
  poly_iter(choice<3>{}, poly, fn);
}

}

#endif // ELEMENTS_HPP