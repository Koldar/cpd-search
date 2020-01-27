#ifndef _CPDSEARCH_CPDJUMPSEARCHLISTENER_HEADER__
#define _CPDSEARCH_CPDJUMPSEARCHLISTENER_HEADER__

#include "CpdSearchListener.hpp"
#include "CpdHeuristic.hpp"

namespace pathfinding::search::listeners {

    using namespace pathfinding;
    using namespace pathfinding::search;

    /**
     * @brief listener for CPD-Jump-Search events
     * 
     * @tparam G 
     * @tparam V 
     * @tparam STATE 
     */
    template <typename G, typename V, typename STATE>
    class CpdJumpSearchListener: public CpdSearchListener<G, V, STATE> {
    public:
        using This = CpdJumpSearchListener<G, V, STATE>;
        using Super = CpdSearchListener<G, V, STATE>;
    public:
        CpdJumpSearchListener() {

        }
        CpdJumpSearchListener(const Super& o): Super{o} {

        }
        virtual ~CpdJumpSearchListener() = default;
        CpdJumpSearchListener(const This& o) = default;
        CpdJumpSearchListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator =(This&& o) = default;
    };

}

#endif