#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <array>
#include "elements.hpp"

namespace utility
{

// a fixed-capacity circular buffer
template <
  typename T,
  size_t max_size
  >
class RingBuffer : public std::array<T,max_size>
{
public:
	RingBuffer() {}
	void put(auto value)
	{
		empty = false;
		tail = full ? inc(tail) : tail;
		(*this)[head] = (T)value;
		head = inc(head);
		full = head == tail;
	}
	T pop()
	{
		full = false;
		auto v = (*this)[tail];
		tail = inc(tail);
		empty = tail == head;
		head = empty ? tail : head;
		return v;
	}
	size_t size() const
	{
		size_t csize = empty ? 0 : max_size;
		if(!full) {
			if(tail > head) {
				csize = max_size - tail + head;
			} else {
				csize = head - tail;
			}
		}
		return csize;
	}
	size_t get_elements(auto &container) const
	{
		auto csize = size();
		auto _tail = tail;
		for(size_t i=0; i<csize; i++) {
			elements::element(container, i) = this->at(_tail);
			_tail = inc(_tail);
		}
		return csize;
	}
	bool is_full() const
	{
		return full;
	}
	bool is_empty() const
	{
		return empty;
	}
private:
	size_t tail{0};
	size_t head{0};
	bool full{0};
	bool empty{true};
private:
	inline
	size_t inc(const size_t &index) const
	{
		return ((index + 1) % max_size);
	}
	inline
	size_t dec(const size_t &index) const
	{
		return ((index + max_size - 1) % max_size);
	}
};

}

#endif // UTILITY_HPP
