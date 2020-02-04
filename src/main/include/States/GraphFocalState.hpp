#ifndef _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__
#define _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__

#include <pathfinding-utils/GraphState.hpp>

#include "cpd_search_generated_e.hpp"
#include "generation_enum_t.hpp"

namespace pathfinding::search {

    /**
     * @brief 
     * 
     * the priority of the state in the open list is available in the field ::priority (from ::GraphState)
     * 
     * @tparam G 
     * @tparam V 
     * @tparam E 
     */
    template <typename G, typename V, typename E>
    class GraphFocalState: public GraphState<G, V, E, generation_enum_t> {
        using This = GraphFocalState<G, V, E>;
        using Super = GraphState<G, V, E, generation_enum_t>;
    private:
        /**
         * @brief if we follow the CPDPath from this search node till the goal, this is the source of the first perturbated edge we encounter
         * 
         */
        nodeid_t lastEarliestPerturbationSourceId;
        /**
         * @brief the cost we need to pay to go directly from this search node to the node just before the first perturbation.
         * 
         */
        cost_t lastEarliestPerturbationSourceIdCost;
    public:
        GraphFocalState(stateid_t id, const IImmutableGraph<G, V, E>& g, nodeid_t location, generation_enum_t source): Super{id, g, location, source}, 
            lastEarliestPerturbationSourceId{0}, lastEarliestPerturbationSourceIdCost{cost_t::INFTY} {

        }
        virtual ~GraphFocalState() {

        }
        GraphFocalState(const This& other) = delete;
        GraphFocalState(This&& other) : Super{::std::move(other)}, 
            lastEarliestPerturbationSourceId{other.lastEarliestPerturbationSourceId}, 
            lastEarliestPerturbationSourceIdCost{other.lastEarliestPerturbationSourceIdCost} {

        }

        This& operator =(const This& other) = delete;
        This& operator =(This&& other) {
            Super::operator =(::std::move(other));
            this->source = other.source;
            this->lastEarliestPerturbationSourceId = other.lastEarliestPerturbationSourceId;
            this->lastEarliestPerturbationSourceIdCost = other.lastEarliestPerturbationSourceIdCost;

            return *this;
        }
    public:
        This* getParent() {
            return static_cast<This*>(this->parent);
        }
        const This* getParent() const {
            return static_cast<const This*>(this->parent);
        }
        void setParent(This* parent) {
            this->parent = static_cast<This*>(parent);
        }
    public:
        void updateEarlyPerturbationInfo(nodeid_t sourceId, cost_t costToReach) {
            this->lastEarliestPerturbationSourceId = sourceId;
            this->lastEarliestPerturbationSourceIdCost = costToReach;
        }
        nodeid_t getEarliestPerturbationSourceId() const {
            return this->lastEarliestPerturbationSourceId;
        }
        cost_t getCostToEarliestPerturbationSourceId() const {
            return this->lastEarliestPerturbationSourceIdCost;
        }
    public:
        friend bool operator ==(const This& a, const This& b) {
            return a.id == b.id && a.position == b.position;
        }
        friend bool operator !=(const This& a, const This& b) {
            return !(a == b);
        }
        friend std::ostream& operator <<(std::ostream& out, const This& state) {
            out 
                << "{ id: " 
                << state.getId() 
                << " node: " 
                << state.graph.getVertex(state.getPosition())
                << " f: "
                << state.getF()
                << " g: "
                << state.getG()
                << " }";
            return out;
        }
    };



}

#endif