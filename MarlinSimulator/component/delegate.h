#ifndef DELEGATE_H
#define DELEGATE_H

#include <stddef.h>
/* Here be dragons... */
/* TODO: EM-2762 [New] - This can pretty much be replaced with std::function<> 
   as this is a less functional, less flexible form of std::function
*/

template<typename A0=void, typename A1=void>
class Delegate
{
public:
    Delegate() : object_ptr(NULL), stub_ptr(NULL)
    {
    }

    template <class T, void (T::*TMethod)(A0 a0, A1 a1)>
    static Delegate from_method(T* object_ptr)
    {
        Delegate d;
        d.object_ptr = object_ptr;
        d.stub_ptr = &method_stub<T, TMethod>;
        return d;
    }

    void operator()(A0 a0, A1 a1) const
    {
        if (!object_ptr)
            return;
        return stub_ptr(object_ptr, a0, a1);
    }
    
    operator bool() const { return object_ptr != NULL; }

private:
    typedef void (*stub_type)(void* object_ptr, A0 a0, A1 a1);

    void* object_ptr;
    stub_type stub_ptr;
    
    template <class T, void (T::*TMethod)(A0 a0, A1 a1)>
    static void method_stub(void* object_ptr, A0 a0, A1 a1)
    {
        T* p = static_cast<T*>(object_ptr);
        return (p->*TMethod)(a0, a1);
    }
};

#define DELEGATE(DelegateType, Type, Object, Method) (DelegateType::from_method<Type, &Type::Method>(&Object))
              
#endif//DELEGATE_H
