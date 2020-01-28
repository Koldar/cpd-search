#ifndef _CPDSEARCH_CPDJUMPTRAILSEARCHLISTENER_HEADER__
#define _CPDSEARCH_CPDJUMPTRAILSEARCHLISTENER_HEADER__

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
    class CpdJumpTrailSearchListener: public CpdSearchListener<G, V, STATE> {
    public:
        using This = CpdJumpTrailSearchListener<G, V, STATE>;
        using Super = CpdSearchListener<G, V, STATE>;
    public:
        CpdJumpTrailSearchListener() {

        }
        CpdJumpTrailSearchListener(const Super& o): Super{o} {

        }
        virtual ~CpdJumpTrailSearchListener() = default;
        CpdJumpTrailSearchListener(const This& o) = default;
        CpdJumpTrailSearchListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator =(This&& o) = default;
    };

}

#endif