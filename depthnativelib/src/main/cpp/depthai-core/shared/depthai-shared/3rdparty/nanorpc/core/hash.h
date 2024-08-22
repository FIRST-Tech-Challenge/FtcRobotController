#ifndef __NANO_RPC_CORE_HASH_H__
#define __NANO_RPC_CORE_HASH_H__

// STD
#include <cstdint>
#include <string>

namespace nanorpc
{
namespace core
{
		

inline type::id hash_id(const std::string& str) {
	type::id h = UINT64_C(1125899906842597); // prime
	for(const auto& c : str) h = 31 * h + c; // as 'h' is unsigned, both 31 and 'c' are promoted to unsigned
	return h;
}


}	// namespace core
}	// namespace nanorpc


#endif  // !__NANO_RPC_CORE_HASH_H__
