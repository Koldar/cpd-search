#ifndef _CPD_SEARCH_CPD_FOCAL_EXPANDER_HEADER__
#define _CPD_SEARCH_CPD_FOCAL_EXPANDER_HEADER__

namespace pathfinding::search {

    template <typename G, typename V, typename E>
    class CpdFocalExpander: public StandardStateExpander<CpdFocalState<G, V, E>> {
        typedef CpdFocalExpander<G, V, E> CpdFocalExpanderInstance;
    protected:
        const CpdManager& cpdManager;
    public:
        CpdFocalExpander(const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph, const CpdManager<G, V>& cpdManager): StandardStateExpander<CpdFocalState<G, V, E>>{graph}, cpdManager{cpdManager} {

        }
        CpdFocalExpander(const CpdFocalExpanderInstance& other): StandardStateExpander<CpdFocalState<G, V, E>>{other}, cpdManager{other.cpdManager} {

        }
        CpdFocalExpander(CpdFocalExpanderInstance&& other): StandardStateExpander<CpdFocalState<G, V, E>>{::std::move(other)}, cpdManager{other.cpdManager} {
            
        }
        CpdFocalExpanderInstance& operator =(const CpdFocalExpanderInstance& other) {
            StandardStateExpander<CpdFocalState<G, V, E>>::operator =(other);
            this->cpdManager = other.cpdManager;
            return *this;
        }
        CpdFocalExpanderInstance& operator =(CpdFocalExpanderInstance&& other) {
            StandardStateExpander<CpdFocalState<G, V, E>>::operator =(other);
            this->cpdManager = other.cpdManager;
            return *this;
        }
        ~CpdFocalExpander() {

        }
    public:
        virtual cpp_utils::vectorplus<std::pair<STATE&, cost_t>> getSuccessors(const Graph& state, IStateSupplier<STATE, nodeid_t>& supplier) {
            cpp_utils::vectorplus<std::pair<STATE&, cost_t>> result{};
            //****************** MOVING *********************
            for (auto outEdge : graph.getOutEdges(state.getPosition())) {
                fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
                result.add(std::pair<STATE&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
                    GET_COST_FUNCTION(outEdge.getPayload())
                });
            }
            //

            return result;
        }
        virtual std::pair<STATE&, cost_t> getSuccessor(const STATE& state, int successorNumber, IStateSupplier<STATE, nodeid_t>& supplier) {
            auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
            return std::pair<STATE&, cost_t>{
                supplier.getState(outEdge.getSinkId()),
                GET_COST_FUNCTION(outEdge.getPayload())
            };
        }
    };

}

#endif