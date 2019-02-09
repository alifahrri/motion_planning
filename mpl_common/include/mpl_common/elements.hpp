#ifndef ELEMENTS_HPP
#define ELEMENTS_HPP

#include "choice.hpp"

namespace elements {

template<size_t idx>
inline
auto element(choice<2>, const auto &e) -> decltype(e.at(idx)) {
  return e.at(idx);
}

template<size_t idx>
inline
auto element(choice<1>, const auto &e) -> decltype(e(idx)) {
  return e(idx);
}

template<size_t idx>
inline
auto element(choice<0>, const auto &e) -> decltype(e[idx]) {
  return e[idx];
}

template<size_t idx>
inline
auto element(const auto &e) -> decltype(element<idx>(choice<2>{}, e)) {
	return element<idx>(choice<2>{},e);
}

inline
auto element(choice<2>, const auto &e, size_t idx) -> decltype(e.at(idx)) {
  return e.at(idx);
}

inline
auto element(choice<1>, const auto &e, size_t idx) -> decltype(e(idx)) {
  return e(idx);
}

inline
auto element(choice<0>, const auto &e, size_t idx) -> decltype(e[idx]) {
  return e[idx];
}

inline 
auto element(const auto &e, size_t idx) -> decltype(element(choice<2>{}, e, idx)) {
	return element(choice<2>{}, e, idx);
}

/* get mutable
 */

template<size_t idx>
inline
auto element(choice<2>, auto &e) -> decltype((e.at(idx))) {
  return e.at(idx);
}

template<size_t idx>
inline
auto element(choice<1>, auto &e) -> decltype((e(idx))) {
  return e(idx);
}

template<size_t idx>
inline
auto element(choice<0>, auto &e) -> decltype((e[idx])) {
  return e[idx];
}

template<size_t idx>
inline 
auto element(auto &e) -> decltype((element<idx>(choice<2>{}, e))) {
	return (element<idx>(choice<2>{}, e));
}

inline
auto element(choice<2>, auto &e, size_t idx) -> decltype((e.at(idx))) {
  return e.at(idx);
}

inline
auto element(choice<1>, auto &e, size_t idx) -> decltype((e(idx))) {
  return e(idx);
}

inline
auto element(choice<0>, auto &e, size_t idx) -> decltype((e[idx])) {
  return e[idx];
}

inline
auto element(auto &e, size_t idx) -> decltype((element(choice<2>{}, e, idx))) {
	return (element(choice<2>{}, e, idx));
}

/* generic getter functino for x and y elements
 *
 */

template<size_t>
inline
auto x(choice<2>, const auto &point) -> decltype(point.x()) {
  return point.x();
}

template<size_t>
inline
auto x(choice<1>, const auto &point) -> decltype(point.x) {
  return point.x;
}

template<size_t idx>
inline
auto x(choice<0>, const auto &point) -> decltype(element<idx>(choice<2>{},point)) {
  return element<idx>(choice<2>{},point);
}

template<size_t idx=0>
inline
auto x(const auto &point) -> decltype(x<idx>(choice<2>{},point)) {
  return x<idx>(choice<2>{},point);
}

template<size_t>
inline
auto y(choice<2>, const auto &point) -> decltype(point.y()) {
  return point.y();
}

template<size_t>
inline
auto y(choice<1>, const auto &point) -> decltype(point.y) {
  return point.y;
}

template<size_t idx>
inline
auto y(choice<0>, const auto &point) -> decltype(element<idx>(choice<2>{},point)) {
  return element<idx>(choice<2>{},point);
}

template<size_t idx=1>
inline
auto y(const auto &point) -> decltype(y<idx>(choice<2>{},point)) {
  return y<idx>(choice<2>{},point);
}

template<size_t>
inline
auto z(choice<2>, const auto &point) -> decltype(point.z()) {
  return point.z();
}

template<size_t>
inline
auto z(choice<1>, const auto &point) -> decltype(point.z) {
  return point.z;
}

template<size_t idx>
inline
auto z(choice<0>, const auto &point) -> decltype(element<idx>(choice<2>{},point)) {
  return element<idx>(choice<2>{},point);
}

template<size_t idx=2>
inline
auto z(const auto &point) -> decltype(z<idx>(choice<2>{},point)) {
  return z<idx>(choice<2>{},point);
}

/* get reference of the elements
 */
template<size_t>
inline
auto x(choice<2>, auto &point) -> decltype((point.x())) {
  return point.x();
}

template<size_t>
inline
auto x(choice<1>, auto &point) -> decltype((point.x)) {
  return point.x;
}

template<size_t idx>
inline
auto x(choice<0>, auto &point) -> decltype((element<idx>(choice<2>{},(point)))) {
  return element<idx>(choice<2>{},point);
}

template<size_t idx=0>
inline
auto x(auto &point) -> decltype((x<idx>(choice<2>{},(point)))) {
  return x<idx>(choice<2>{},point);
}

template<size_t>
inline
auto y(choice<2>, auto &point) -> decltype((point.y())) {
  return point.y();
}

template<size_t>
inline
auto y(choice<1>, auto &point) -> decltype((point.y)) {
  return point.y;
}

template<size_t idx>
inline
auto y(choice<0>, auto &point) -> decltype((element<idx>(choice<2>{},(point)))) {
  return element<idx>(choice<2>{},point);
}

template<size_t idx=1>
inline
auto y(auto &point) -> decltype((y<idx>(choice<2>{},(point)))) {
  return y<idx>(choice<2>{},point);
}

template<size_t>
inline
auto z(choice<2>, auto &point) -> decltype((point.z())) {
  return point.z();
}

template<size_t>
inline
auto z(choice<1>, auto &point) -> decltype((point.z)) {
  return point.z;
}

template<size_t idx>
inline
auto z(choice<0>, auto &point) -> decltype((element<idx>(choice<2>{},(point)))) {
  return element<idx>(choice<2>{},point);
}

template<size_t idx=2>
inline
auto z(auto &point) -> decltype((z<idx>(choice<2>{},(point)))) {
  return z<idx>(choice<2>{},point);
}

/* generic getter function for circles
 */
template <size_t>
inline
auto radius(choice<5>, const auto &circle) -> decltype(circle.r()) {
  return circle.r();
}

template <size_t>
inline
auto radius(choice<4>, const auto &circle) -> decltype(circle.r) {
  return circle.r;
}

template <size_t>
inline
auto radius(choice<3>, const auto &circle) -> decltype(circle.radius()) {
  return circle.radius();
}

template <size_t>
inline
auto radius(choice<2>, const auto &circle) -> decltype(circle.radius) {
  return circle.radius;
}

template <size_t>
inline
auto radius(choice<1>, const auto &circle) -> decltype(circle.radius()) {
  return circle.radius();
}

template <size_t r_idx>
inline
auto radius(choice<0>, const auto &circle) -> decltype(element<r_idx>(choice<2>{},(circle))) {
  return element<r_idx>(choice<2>{}, circle);
}

template <size_t r_idx=2>
inline
auto radius(const auto &circle) -> decltype(radius<r_idx>(choice<5>{},circle)) {
  return radius<r_idx>(choice<5>{}, circle);
}

/* generic getter function for mutable circles
 */

template <size_t>
inline
auto radius(choice<5>, auto &circle) -> decltype((circle.r())) {
  return circle.r();
}

template <size_t>
inline
auto radius(choice<4>, auto &circle) -> decltype((circle.r)) {
  return circle.r;
}

template <size_t>
inline
auto radius(choice<3>, auto &circle) -> decltype((circle.radius())) {
  return circle.radius();
}

template <size_t>
inline
auto radius(choice<2>, auto &circle) -> decltype((circle.radius)) {
  return circle.radius;
}

template <size_t>
inline
auto radius(choice<1>, auto &circle) -> decltype((circle.radius())) {
  return circle.radius();
}

template <size_t r_idx>
inline
auto radius(choice<0>, auto &circle) -> decltype((element<r_idx>(choice<2>{},(circle)))) {
  return element<r_idx>(choice<2>{}, circle);
}

template <size_t r_idx=2>
inline
auto radius(auto &circle) -> decltype((radius<r_idx>(choice<5>{},circle))) {
  return radius<r_idx>(choice<5>{}, circle);
}

/* generic getter functino for points of line
 *
 */
template<size_t>
inline
auto p0(choice<2>, const auto &line) -> decltype(line.p0()) {
  return line.p0();
}

template<size_t>
inline
auto p0(choice<1>, const auto &line) -> decltype(line.p0) {
  return line.p0;
}

template<size_t idx>
inline
auto p0(choice<0>, const auto &line) -> decltype(element<idx>(choice<2>{},line)) {
  return element<idx>(choice<2>{}, line);
}

template<size_t idx=0>
inline
auto p0(const auto &line) -> decltype(p0<idx>(choice<2>{}, line)) {
  return p0<idx>(choice<2>{},line);
}

template<size_t>
inline
auto p1(choice<2>, const auto &line) -> decltype(line.p1()) {
  return line.p1();
}

template<size_t>
inline
auto p1(choice<1>, const auto &line) -> decltype(line.p1) {
  return line.p1;
}

template<size_t idx>
inline
auto p1(choice<0>, const auto &line) -> decltype(element<idx>(choice<2>{}, line)) {
  return element<idx>(choice<2>{}, line);
}

template<size_t idx=1>
inline
auto p1(const auto &line) -> decltype(p1<idx>(choice<2>{}, line)) {
  return p1<idx>(choice<2>{},line);
}

/* generic getter functino for mutable points of line
 *
 */
template<size_t>
inline
auto p0(choice<2>, auto &line) -> decltype((line.p0())) {
  return line.p0();
}

template<size_t>
inline
auto p0(choice<1>, auto &line) -> decltype((line.p0)) {
  return line.p0;
}

template<size_t idx>
inline
auto p0(choice<0>, auto &line) -> decltype((element<idx>(choice<2>{}, line))){
  return element<idx>(choice<2>{}, line);
}

template<size_t idx=0>
inline
auto p0(auto &line) -> decltype((p0<idx>(choice<2>{},line))) {
  return p0<idx>(choice<2>{},line);
}

template<size_t>
inline
auto p1(choice<2>, auto &line) -> decltype((line.p1())) {
  return line.p1();
}

template<size_t>
inline
auto p1(choice<1>, auto &line) -> decltype((line.p1)) {
  return line.p1;
}

template<size_t idx>
inline
auto p1(choice<0>, auto &line) -> decltype((element<idx>(choice<2>{}, line))){
  return element<idx>(choice<2>{}, line);
}

template<size_t idx=1>
inline
auto p1(auto &line) -> decltype((p1<idx>(choice<2>{},line))) {
  return p1<idx>(choice<2>{},line);
}

/* Generic getter function for line from polygon
 */

inline
auto line(choice<2>, const auto &poly, size_t i) -> decltype(element(choice<2>{}, poly.lines, i)) {
  return element(choice<2>{}, poly.lines, i);
}

inline
auto line(choice<1>, const auto &poly, size_t i) -> decltype(element(choice<2>{}, poly.lines(), i)) {
  return element(choice<2>{}, poly.lines(), i);
}

inline
auto line(choice<0>, const auto &poly, size_t i) -> decltype(element(choice<2>{}, poly, i)) {
  return element(choice<2>{}, poly, i);
}

inline
auto line(const auto &poly, size_t i) -> decltype(line(choice<2>{}, poly, i)) {
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
auto poly_iter(const auto &poly, auto &fn) -> decltype(poly_iter(choice<3>{}, poly, fn), void()) {
  poly_iter(choice<3>{}, poly, fn);
}

}

#endif // ELEMENTS_HPP