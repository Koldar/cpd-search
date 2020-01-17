#ifndef _CPDSEARCH_CPDSEARCHLISTENER_HEADER__
#define _CPDSEARCH_CPDSEARCHLISTENER_HEADER__

#include <pathfinding-utils/AstarListener.hpp>
#include <pathfinding-utils/GraphState.hpp>
#include <cpd-search/PerturbatedCost.hpp>

namespace pathfinding::search {

    /**
     * @brief class which listen for events in a CPDSearch run
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V, typename STATE>
    class CpdSearchListener: public AstarListener<STATE> {
    public:
        typedef CpdSearchListener<G, V, STATE> This;
        typedef AstarListener<STATE> Super;
    public:
        CpdSearchListener() {

        }
        CpdSearchListener(const Super& o): Super{o} {

        }
        virtual ~CpdSearchListener() = default;
        CpdSearchListener(const This& o) = default;
        CpdSearchListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator =(This&& o) = default;
    public:
        /**
         * @brief callback create when CPD search activates it early termination mechanism
         * 
         * @param from the node where we start early terminating.
         */
        virtual void onEarlyTerminationActivated(const STATE& from, const STATE& goal) = 0;

        /**
         * @brief callback invoked when the upperbound mantained by CPD search is revised
         * 
         * The method is invoke before actually revising the upperbound
         * 
         * @param from the state that trigger the update of the upperbound
         * @param oldUpperbound upperbound before the update
         * @param newUpperbound upperbound after the update
         */
        virtual void onUpperboundRevised(const STATE& from, cost_t oldUpperbound, cost_t newUpperbound) = 0;
    };

}

#endif