#ifndef _CPDSEARCH_CPDFOCALSEARCHLISTENER_HEADER__
#define _CPDSEARCH_CPDFOCALSEARCHLISTENER_HEADER__

#include "CpdSearchListener.hpp"

namespace pathfinding::search::listeners {

    template <typename G, typename V, typename STATE>
    class CpdFocalSearchListener: public CpdSearchListener<G, V, STATE> {
    public:
        using This = CpdFocalSearchListener<G, V, STATE>;
        using Super = CpdSearchListener<G, V, STATE>;
    public:
        CpdFocalSearchListener() {

        }
        CpdFocalSearchListener(const Super& o): Super{o} {

        }
        virtual ~CpdFocalSearchListener() = default;
        CpdFocalSearchListener(const This& o) = default;
        CpdFocalSearchListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator =(This&& o) = default;
    };

}

#endif