#ifndef _CPDSEARCH_CPDEXPANDER_HEADER__
#define _CPDSEARCH_CPDEXPANDER_HEADER__

#include <cpp-utils/igraph.hpp>


#include "PerturbatedCost.hpp"
#include "cpd_search_generated_e.hpp"

namespace pathfinding::search {

    /**
     * @brief a state expander that not only generate up to 8 successors of a node depending on the vertex successors of an underlying graph, but generates a state by following the cpd-path of the cpd up until it encounters its first perturbation
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V, typename STATE>
    class CpdExpander: public IStateExpander<STATE, nodeid_t> {
    public:
        using This = CpdExpander<G,V,STATE>;
        using Super = IStateExpander<STATE, nodeid_t>;
        using Supplier = IStateSupplier<STATE, nodeid_t>;
    protected:
        const cpp_utils::graphs::IImmutableGraph<G, V, PerturbatedCost>& graph;
    public:
        CpdExpander(const cpp_utils::graphs::IImmutableGraph<G, V, PerturbatedCost>& graph): graph{graph} {

        }
        CpdExpander(const This& other) = delete;
        CpdExpander(This&& other): graph{other.graph} {
        }
        This& operator =(const This& other) = delete;
        This& operator =(This&& other) {
            this->graph = other.graph;
            return *this;
        }
        virtual ~CpdExpander() {

        }
    public:
        virtual cpp_utils::vectorplus<std::pair<STATE&, cost_t>> getSuccessors(const STATE& state, Supplier& supplier) {
            cpp_utils::vectorplus<std::pair<STATE&, cost_t>> result{};

            //****************** FOLLOW CPD UNTIL IT FINDS EITHER THE GOAL OR A PERTURBATION ********************
            STATE& beforePerturbation = supplier.getState(state.getEarliestPerturbationSourceId());
            critical("follow the cpd path from ", state, "till", beforePerturbation, "cost to reach it is", state.getCostToEarliestPerturbationSourceId());
            result.add(std::pair<STATE&, cost_t>{
                beforePerturbation,
                state.getCostToEarliestPerturbationSourceId()
            });
            nodeid_t nodeFollowingCPDPath = beforePerturbation.getId();
            //****************** MOVING *********************
            for (auto outEdge : this->graph.getOutEdges(state.getPosition())) {
                critical("an outedge ", outEdge, " of ", this->graph.getVertex(state.getPosition()), "(", &state, ") goes to", this->graph.getVertex(outEdge.getSinkId()), "edge payload of", outEdge.getPayload());
                if (outEdge.getSinkId() == nodeFollowingCPDPath) {
                    //we have already generated this state. Ignore it
                    continue;
                }
                result.add(std::pair<STATE&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
                    outEdge.getPayload().getCost()
                });
            }

            critical(state, "has ", result.size(), "successors!");
            return result;
        }
        //TODO here we require that the state has getEarliestPerturbationSourceId method. It would be better to create a method for it or a new subclass of STATE where this method is added as a pure virtual method.
        virtual std::pair<STATE&, cost_t> getSuccessor(const STATE& state, int successorNumber, Supplier& supplier) {
            if (successorNumber == this->graph.getOutDegree(state.getPosition())) {
                return std::pair<STATE&, cost_t>{
                    supplier.getState(state.getEarliestPerturbationSourceId()),
                    state.getCostToEarliestPerturbationSourceId()
                };   
            } else {
                auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
                return std::pair<STATE&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
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