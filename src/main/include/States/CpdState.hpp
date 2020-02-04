#ifndef _CPDSEARCH_CPDSTATE_HEADER__
#define _CPDSEARCH_CPDSTATE_HEADER__

#include <pathfinding-utils/GraphState.hpp>

namespace pathfinding::search {

    /**
     * @brief a state which knows that there is an underlygin CPD over the graph
     * 
     * Thanks to the CPD, along the other properties, each state know what is the id where the next perturbation is alongside the cost of the path to optimally reach it
     * 
     * @tparam G type of the payload of the whole graph
     * @tparam V type of the each vertex in the graph
     * @tparam E type of each edge in the graph
     */
    template <typename G, typename V, typename E, typename REASON>
    class CpdState: public GraphState<G, V, E, REASON> {
    public:
        using This = CpdState<G, V, E, REASON>;
        using Super = GraphState<G, V, E, REASON>;
    protected:
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
         * @brief actual cost till the goal
         * 
         */
        cost_t perturbatedPathCost;
    public:
        CpdState(stateid_t id, const IImmutableGraph<G, V, E>& g, nodeid_t location): Super{id, g, location}, 
            lastEarliestPerturbationSourceId{0}, lastEarliestPerturbationSourceIdCost{cost_t::INFTY}, perturbatedPathCost{cost_t::INFTY} {

        }
        virtual ~CpdState() {

        }
        CpdState(const This& other) = delete;
        CpdState(This&& other) : Super{::std::move(other)}, 
            lastEarliestPerturbationSourceId{other.lastEarliestPerturbationSourceId}, 
            lastEarliestPerturbationSourceIdCost{other.lastEarliestPerturbationSourceIdCost},
            perturbatedPathCost{other.perturbatedPathCost} {

        }

        This& operator =(const This& other) = delete;
        This& operator =(This&& other) {
            Super::operator =(::std::move(other));
            this->lastEarliestPerturbationSourceId = other.lastEarliestPerturbationSourceId;
            this->lastEarliestPerturbationSourceIdCost = other.lastEarliestPerturbationSourceIdCost;
            this->perturbatedPathCost = other.perturbatedPathCost;

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
        void updatePerturbatedPathCostToGoal(cost_t perturbatedPathCost) {
            this->perturbatedPathCost = perturbatedPathCost;
        }
        nodeid_t getEarliestPerturbationSourceId() const {
            return this->lastEarliestPerturbationSourceId;
        }
        cost_t getCostToEarliestPerturbationSourceId() const {
            return this->lastEarliestPerturbationSourceIdCost;
        }
    public:
        friend bool operator <(const This& a, const This& b) {
            //used in open list
            if (a.getF() < b.getF()) {
                return true;
            } else if (a.getF() == b.getF()) {
                //tie breaking. Choose the state with maximum g
                //return a.getG() > b.getG();
                //tie breaking. Choose the state with smaller cpd perturbated cost
                return a.perturbatedPathCost < b.perturbatedPathCost;
            } else {
                return false;
            }
        }
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