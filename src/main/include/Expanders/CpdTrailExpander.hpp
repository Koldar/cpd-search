#ifndef _CPDSEARCH_CPDTRAILEXPANDER_HEADER__
#define _CPDSEARCH_CPDTRAILEXPANDER_HEADER__

#include <cpp-utils/igraph.hpp>
#include <cpp-utils/math.hpp>
#include <cpp-utils/exceptions.hpp>

#include "CpdFocalHeuristic.hpp"
#include "PerturbatedCost.hpp"
#include "cpd_search_generated_e.hpp"

namespace pathfinding::search {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;

    /**
     * @brief a state expander that not only generate up to 8 successors of a node depending on the vertex successors of an underlying graph, but generates all the states obtain by iteratively following the cpd-path of the cpd up until it encounters its first perturbation.
     * 
     * The expander automatically set such discount. Undiscounted states has discount value of 1
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V, typename STATE>
    class CpdTrailExpander: public IStateExpander<STATE, nodeid_t, cpd_search_generated_e> {
    public:
        using This = CpdTrailExpander<G,V,STATE>;
        using Super = IStateExpander<STATE, nodeid_t, cpd_search_generated_e>;
        using Supplier = IStateSupplier<STATE, nodeid_t, cpd_search_generated_e>;
    protected:
        const cpp_utils::graphs::IImmutableGraph<G, V, PerturbatedCost>& graph;
        /**
         * @brief the cpd heuristic used to fetch the states between one node and the other by following the cpd path
         * 
         */
        CpdFocalHeuristic<STATE, G, V>& cpdHeuristic;
        double minDiscount;
        double maxDiscount;
        /**
         * @brief a coefficient repersents how long it takes to dissipate the discount from the parent.
         * 
         * values near 0 means that the discount will take a while in order to be set to 1 while values near 1 represents discount that quickly dissipates.
         */
        double discountDecay;
    public:
        CpdTrailExpander(const cpp_utils::graphs::IImmutableGraph<G, V, PerturbatedCost>& graph, CpdFocalHeuristic<STATE, G, V>& cpdHeuristic, double minDiscount, double maxDiscount, double discountDecay): graph{graph}, cpdHeuristic{cpdHeuristic}, minDiscount{minDiscount}, maxDiscount{maxDiscount}, discountDecay{discountDecay} {

        }
        CpdTrailExpander(const This& other) = delete;
        CpdTrailExpander(This&& other): graph{other.graph}, cpdHeuristic{other.cpdHeuristic}, minDiscount{other.minDiscount}, maxDiscount{other.maxDiscount}, discountDecay{std::move(other.discountDecay)} {
        }
        This& operator =(const This& other) = delete;
        This& operator =(This&& other) {
            this->graph = other.graph;
            this->cpdHeuristic = cpdHeuristic;
            this->minDiscount = std::move(other.minDiscount);
            this->maxDiscount = std::move(other.maxDiscount);
            this->discountDecay = std::move(other.discountDecay);
            return *this;
        }
        virtual ~CpdTrailExpander() {

        }
    public:
        virtual cpp_utils::vectorplus<std::pair<STATE&, cost_t>> getSuccessors(const STATE& state, Supplier& supplier) {
            cpp_utils::vectorplus<std::pair<STATE&, cost_t>> result{};

            nodeid_t firstSuccessorAlongCpdPath = this->graph.size() + 1;

            double stateDiscount = state.getDiscount();

            //****************** FOLLOW CPD UNTIL IT FINDS EITHER THE GOAL OR A PERTURBATION ********************
            STATE* previous = &const_cast<STATE&>(state);
            //the cost between state "state" and the current element of the trail. 
            cost_t costFromStateToCurrentTrail = 0;
            while(true) {
                //get next state
                auto edge = this->graph.getOutEdge(
                    previous->getPosition(), 
                    this->cpdHeuristic.getNextMove(previous->getPosition())
                );
                if (edge.getPayload().isPerturbated()) {
                    //found the first perturbation. end loop
                    break;
                }
                if (result.isEmpty()) {
                    //the first move we perform generates a state. Cache it because this is the only state we may duplicate in this list
                    firstSuccessorAlongCpdPath = edge.getSinkId();
                }
                /* while we are computing the intermediate node, we need to set not only cost from the state value, but the parent as well.
                 * which parent should we set? 2 choices:
                 *  - the state where the early termination has started: if we choose this one, the solution path will lack all the intemediate nodes (hence, not a great solution)
                 *  - the state before this one in the cpd path
                 */
                STATE& trailState = supplier.getState(edge.getSinkId(), cpd_search_generated_e::CPDPATH);
                cost_t cost = edge.getPayload().getCost();
                finer("follow the cpd path from ", *previous, "we reached", trailState, "cost to reach it is", cost);
                costFromStateToCurrentTrail += cost;
                result.add(std::pair<STATE&, cost_t>{
                    trailState,
                    costFromStateToCurrentTrail
                });
                previous = &trailState;
            }
            //update the discount factor for the generated states
            // costFromStateToCurrentTrail represents the distance to the endpoint of the earliest perturbation!
            // the state generated by following the cpd has their discount resetted
            //if the state already exists, the discount is updated only if it's better. Otherwise, we won't update it
            //the max discount is not maxDiscount, but the state current discount
            auto m1 = getM(0, stateDiscount, costFromStateToCurrentTrail, this->minDiscount);
            auto q1 = getQ(0, stateDiscount, m1);
            auto_debug(costFromStateToCurrentTrail);
            auto_debug(stateDiscount);
            auto_debug(this->minDiscount);
            auto_debug(m1);
            auto_debug(q1);
            for (int i=result.lastIndex(); i>=0; --i) {
                info("analyzing cpd-path temp state", result[i].first);
                auto oldDiscount = result[i].first.getDiscount();

                auto_debug(oldDiscount);
                //the more states are near the starting state, the more the discount is near 1. The more are near the earliest perturbation, the more the discount is near 0
                auto newDiscount = linearTransform(result[i].second, m1, q1) * state.getDiscount();
                auto_debug(newDiscount);
                newDiscount = bound(newDiscount, this->minDiscount, this->maxDiscount);
                auto_debug(newDiscount);
                
                if (newDiscount < oldDiscount) {
                    result[i].first.setDiscount(newDiscount);
                }
            }
            //****************** MOVING *********************
            //these states will have its discount incremented
            //if the state already exists, the discount is updated only 
            //we need to create states where the discount increases since we are not exploiting cpd paths.
            //the minimum discount is not minDiscount, but the current state discount
            auto m2 = getM(stateDiscount, stateDiscount, this->maxDiscount, this->maxDiscount);
            auto q2 = getQ(stateDiscount, stateDiscount, m2);
            bool maxed = isApproximatelyEqual(stateDiscount, 1., 1e-3);
            auto_debug(stateDiscount);
            auto_debug(this->minDiscount);
            auto_debug(this->maxDiscount);
            auto_debug(m2);
            auto_debug(q2);
            for (auto outEdge : this->graph.getOutEdges(state.getPosition())) {
                fine("an outedge ", outEdge, " of ", this->graph.getVertex(state.getPosition()), "(", &state, ") goes to", this->graph.getVertex(outEdge.getSinkId()), "edge payload of", outEdge.getPayload());
                if (outEdge.getSinkId() == firstSuccessorAlongCpdPath) {
                    //we have already generated this state. Ignore it
                    continue;
                }
                info("analyzing single moves");
                STATE& successor = supplier.getState(outEdge.getSinkId(), cpd_search_generated_e::GRAPHSUCCESSOR);
                if (maxed) {
                    //we are already at 1
                    successor.setDiscount(1.);
                } else {
                    auto oldDiscount = successor.getDiscount();
                    auto_debug(oldDiscount);
                    auto newDiscount = getMonotonicallyCrescentFast(
                        oldDiscount, this->discountDecay, 
                        m2, q2,
                        this->minDiscount, this->maxDiscount
                    );
                    auto_debug(newDiscount);
                    newDiscount = bound(newDiscount, this->minDiscount, this->maxDiscount);
                    auto_debug(newDiscount);
                    if (newDiscount < oldDiscount) {
                        successor.setDiscount(newDiscount);
                    }
                }

                result.add(std::pair<STATE&, cost_t>{
                    successor,
                    outEdge.getPayload().getCost()
                });
            }

            finer(state, "has ", result.size(), "successors!");
            return result;
        }
        //TODO here we require that the state has getEarliestPerturbationSourceId method. It would be better to create a method for it or a new subclass of STATE where this method is added as a pure virtual method.
        virtual std::pair<STATE&, cost_t> getSuccessor(const STATE& state, int successorNumber, Supplier& supplier) {
            if (successorNumber >= this->graph.getOutDegree(state.getPosition())) {
                throw cpp_utils::exceptions::makeImpossibleException("success", successorNumber, "is greater than the out degree!");
            }
            auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
            return std::pair<STATE&, cost_t>{
                supplier.getState(outEdge.getSinkId(), cpd_search_generated_e::GRAPHSUCCESSOR),
                outEdge.getPayload().getCost()
            };
        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
    public:
        virtual void cleanup() {

        }
    };

}

#endif