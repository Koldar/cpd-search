#ifndef _CPD_SEARCH_CPD_FOCAL_EXPANDER_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_EXPANDER_HEADER__

#include "GraphFocalState.hpp"
#include "PerturbatedCost.hpp"
#include <pathfinding-utils/StandardStateExpander.hpp>

namespace pathfinding::search {

    template <typename G, typename V>
    class CpdFocalExpander: public StandardStateExpander<GraphFocalState<G, V, PerturbatedCost>, G, V, PerturbatedCost, PerturbatedCost::getCost> {
        typedef CpdFocalExpander<G, V> CpdFocalExpanderInstance;
        typedef StandardStateExpander<GraphFocalState<G, V, PerturbatedCost>, G, V, PerturbatedCost, PerturbatedCost::getCost> Super;
    public:
        CpdFocalExpander(const cpp_utils::graphs::IImmutableGraph<G, V, PerturbatedCost>& graph): Super{graph} {

        }
        CpdFocalExpander(const CpdFocalExpanderInstance& other) = delete;
        CpdFocalExpander(CpdFocalExpanderInstance&& other): Super{::std::move(other)} {
            
        }
        CpdFocalExpanderInstance& operator =(const CpdFocalExpanderInstance& other) = delete;
        CpdFocalExpanderInstance& operator =(CpdFocalExpanderInstance&& other) {
            Super::operator =(other);
            return *this;
        }
        ~CpdFocalExpander() {

        }
    public:
        virtual cpp_utils::vectorplus<std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>> getSuccessors(const GraphFocalState<G, V, PerturbatedCost>& state, IStateSupplier<GraphFocalState<G, V, PerturbatedCost>, nodeid_t>& supplier) {
            cpp_utils::vectorplus<std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>> result{};
            //****************** MOVING *********************
            for (auto outEdge : this->graph.getOutEdges(state.getPosition())) {
                fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
                result.add(std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
                    GET_COST_FUNCTION(outEdge.getPayload())
                });
            }
            //****************** FOLLOW CPD UNTIL IT FINDS EITHER THE GOAL OR A PERTURBATION ********************
            result.add(std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                supplier.getState(state.getEarliestPerturbationSourceId()),
                state.getCostToEarliestPerturbationSourceId()
            });

            return result;
        }
        virtual std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t> getSuccessor(const GraphFocalState<G, V, PerturbatedCost>& state, int successorNumber, IStateSupplier<GraphFocalState<G, V, PerturbatedCost>, nodeid_t>& supplier) {
            if (successorNumber == this.graph.getOutDegree()) {
                return std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                    supplier.getState(state.getEarliestPerturbationSourceId()),
                    state.getCostToEarliestPerturbationSourceId()
                };   
            } else {
                auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
                return std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
                    GET_COST_FUNCTION(outEdge.getPayload())
                };
            }
        }
    };

}

#endif