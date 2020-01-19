#ifndef _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__
#define _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__

#include <pathfinding-utils/GraphState.hpp>

namespace pathfinding::search {

    /**
     * @brief Specifies who has generated the state
     * 
     */
    enum class generation_enum_t {
        /**
         * @brief A* generated this
         * 
         */
        FROM_SEARCH,
        /**
         * @brief the processof generating the actual path from A* search nodes generated this state
         * 
         */
        FROM_SOLUTION,
        /**
         * @brief early termination has generated this
         * 
         */
        FROM_EARLY_TERMINATION,
        /**
         * @brief either the start or the goal state
         * 
         */
        FROM_INPUT
    };

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
    class GraphFocalState: public GraphState<G, V, E> {
        typedef GraphFocalState<G, V, E> This;
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
        /**
         * @brief Speficies who has generated this state
         * 
         */
        generation_enum_t source;
        
    public:
        GraphFocalState(stateid_t id, const IImmutableGraph<G, V, E>& g, nodeid_t location, generation_enum_t source): GraphState<G, V, E>{id, g, location}, 
            source{source},
            lastEarliestPerturbationSourceId{0}, lastEarliestPerturbationSourceIdCost{cost_t::INFTY} {

        }
        virtual ~GraphFocalState() {

        }
        GraphFocalState(const This& other) = delete;
        GraphFocalState(This&& other) : GraphState<G, V, E>{::std::move(other)}, 
            source{other.source},
            lastEarliestPerturbationSourceId{other.lastEarliestPerturbationSourceId}, 
            lastEarliestPerturbationSourceIdCost{other.lastEarliestPerturbationSourceIdCost} {

        }

        This& operator =(const This& other) = delete;
        This& operator =(This&& other) {
            GraphState<G, V, E>::operator =(::std::move(other));
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
        generation_enum_t getSource() const {
            return this->source;
        }
        void setSource(generation_enum_t source) {
            this->source = source;
        }
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