#ifndef _CPD_SEARCH_CPD_FOCAL_EXPANDER_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_EXPANDER_HEADER__

#include "GraphFocalState.hpp"
#include "PerturbatedCost.hpp"
#include <pathfinding-utils/StandardStateExpander.hpp>

namespace pathfinding::search {

    /**
     * @brief a state expander that not only generate up to 8 successors of a node depending on the vertex successors of an underlying graph, but generates a state by following the cpd-path of the cpd up until it encounters its first perturbation
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V>
    class CpdFocalExpander: public IStateExpander<GraphFocalState<G, V, PerturbatedCost>, nodeid_t, cpd_search_generated_e> {
        typedef CpdFocalExpander<G, V> CpdFocalExpanderInstance;
    protected:
        const cpp_utils::graphs::IImmutableGraph<G, V, PerturbatedCost>& graph;
    public:
        CpdFocalExpander(const cpp_utils::graphs::IImmutableGraph<G, V, PerturbatedCost>& graph): graph{graph} {

        }
        CpdFocalExpander(const CpdFocalExpanderInstance& other) = delete;
        CpdFocalExpander(CpdFocalExpanderInstance&& other): graph{other.graph} {
        }
        CpdFocalExpanderInstance& operator =(const CpdFocalExpanderInstance& other) = delete;
        CpdFocalExpanderInstance& operator =(CpdFocalExpanderInstance&& other) {
            this->graph = other.graph;
            return *this;
        }
        virtual ~CpdFocalExpander() {

        }
    public:
        virtual cpp_utils::vectorplus<std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>> getSuccessors(const GraphFocalState<G, V, PerturbatedCost>& state, IStateSupplier<GraphFocalState<G, V, PerturbatedCost>, nodeid_t, cpd_search_generated_e>& supplier) {
            cpp_utils::vectorplus<std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>> result{};

            //****************** FOLLOW CPD UNTIL IT FINDS EITHER THE GOAL OR A PERTURBATION ********************
            result.add(std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                supplier.getState(state.getEarliestPerturbationSourceId(), cpd_search_generated_e::GRAPHSUCCESSOR),
                state.getCostToEarliestPerturbationSourceId()
            });
            nodeid_t nodeFollowingCPDPath = state.getEarliestPerturbationSourceId();
            //****************** MOVING *********************
            for (auto outEdge : this->graph.getOutEdges(state.getPosition())) {
                fine("an outedge ", outEdge, " of ", this->graph.getVertex(state.getPosition()), "(", &state, ") goes to", this->graph.getVertex(outEdge.getSinkId()), "edge payload of", outEdge.getPayload());
                if (outEdge.getSinkId() == nodeFollowingCPDPath) {
                    //we have already generated this state. Ignore it
                    continue;
                }
                result.add(std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                    supplier.getState(outEdge.getSinkId(), cpd_search_generated_e::GRAPHSUCCESSOR),
                    outEdge.getPayload().getCost()
                });
            }

            return result;
        }
        virtual std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t> getSuccessor(const GraphFocalState<G, V, PerturbatedCost>& state, int successorNumber, IStateSupplier<GraphFocalState<G, V, PerturbatedCost>, nodeid_t, cpd_search_generated_e>& supplier) {
            if (successorNumber == this->graph.getOutDegree(state.getPosition())) {
                return std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                    supplier.getState(state.getEarliestPerturbationSourceId(), cpd_search_generated_e::GRAPHSUCCESSOR),
                    state.getCostToEarliestPerturbationSourceId()
                };   
            } else {
                auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
                return std::pair<GraphFocalState<G, V, PerturbatedCost>&, cost_t>{
                    supplier.getState(outEdge.getSinkId(), cpd_search_generated_e::GRAPHSUCCESSOR),
                    outEdge.getPayload().getCost()
                };
            }
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