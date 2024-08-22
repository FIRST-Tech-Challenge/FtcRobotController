// https://herbsutter.com/gotw/_101/
#pragma once

#include <utility>

namespace dai {

template<typename T>
Pimpl<T>::Pimpl() : m{ new T{} } { }

template<typename T>
template<typename ...Args>
Pimpl<T>::Pimpl( Args&& ...args )
    : m{ new T{ std::forward<Args>(args)... } } { }

template<typename T>
Pimpl<T>::~Pimpl() { }

template<typename T>
T* Pimpl<T>::operator->() { return m.get(); }

template<typename T>
T& Pimpl<T>::operator*() { return *m.get(); }

} // namespace dai
