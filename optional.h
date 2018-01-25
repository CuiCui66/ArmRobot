#pragma once
#include <utility>

struct nullopt_t{};

const nullopt_t nullopt{};


template<typename T>
class optional{
    union {T t;};
    bool isHere;
    optional(const T& t): t(t), isHere(true){}
    optional(T&& t): t(std::move(t)), isHere(true){}
    optional(nullopt_t): isHere(false){}
    optional() : isHere(false){}
    optional(const optional& opt): isHere(opt.isHere){
        if(isHere) new(&t) T(opt.t);
    }
    optional(optional&& opt): isHere(opt.isHere){
        if(isHere) new(&t) T(std::move(opt.t));
    }
    ~optional(){
        if(isHere) t.~T();
    }
    optional& operator=(T nt){
        ~optional();
        isHere = true;
        new(&t) T(std::move(nt));
        return *this;
    }
    optional& operator=(nullopt_t){
        ~optional();
        isHere = false;
        return *this;
    }
    optional& operator=(optional opt){
        ~optional();
        isHere = opt.isHere;
        if(isHere) new(&t) T(std::move(opt.t));
    }
    const T* operator ->() const {
        return &t;
    }
    T* operator ->(){
        return &t;
    }
    const T& operator*()const &{
        return t;
    }
    T& operator*() &{
        return t;
    }
    const T&& operator*()const &&{
        return t;
    }
    T&& operator*() &&{
        return t;
    }
    bool has_value() const noexcept{
        return isHere;
    }
    explicit operator bool() const noexcept{
        return isHere;
    }
    T& value() &{
        assert(isHere);
        return t;
    }
    constexpr const T & value() const &{
        assert(isHere);
        return t;
    }
    T&& value() &&{
        assert(isHere);
        return t;
    }
    const T&& value() const &&{
        assert(isHere);
        return t;
    }
    template<typename U>
    constexpr T value_or( U&& default_value ) const&{
        bool(*this) ? **this : static_cast<T>(std::forward<U>(default_value));
    }
    template<typename U >
    constexpr T value_or( U&& default_value ) &&{
        bool(*this) ? std::move(**this) : static_cast<T>(std::forward<U>(default_value));
    }
    void reset() noexcept{
        ~optional();
        isHere = false;
    }
    template<class... Args>
    T& emplace(Args&&... args){
        reset();
        new(&t) T(std::forward<Args>(args)...);
        isHere = true;
    }

};
