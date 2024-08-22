// https://herbsutter.com/gotw/_101/

#pragma once

#include <memory>

namespace dai {

template <typename T>
class Pimpl {
   private:
    std::unique_ptr<T> m;

   public:
    Pimpl();
    template <typename... Args>
    Pimpl(Args&&...);
    ~Pimpl();
    T* operator->();
    T& operator*();
};

}  // namespace dai
