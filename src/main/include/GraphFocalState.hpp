#ifndef _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__
#define _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__

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
        typedef GraphFocalState<G, V, E> GraphFocalStateInstance;
    private:
        /**
         * @brief reference to an openlist
         * 
         */
        const std::shared_ptr<StaticPriorityQueue<GraphFocalStateInstance>>& openList;
        /**
         * @brief reference to a focal list
         * 
         */
        const std::shared_ptr<StaticPriorityQueue<GraphFocalStateInstance>>& focalList;
        /**
         * @brief priority of the node in the focal list
         * 
         */
        priority_t focalListPriority;
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
        GraphFocalState(stateid_t id, const IImmutableGraph<G, V, E>& g, nodeid_t location, const std::shared_ptr<StaticPriorityQueue<GraphFocalStateInstance>>& openList, const std::shared_ptr<StaticPriorityQueue<GraphFocalStateInstance>>& focusList, generation_enum_t source): GraphState<G, V, E>{id, g, location}, 
            openList{openList}, focalList{focalList}, 
            source{source},
            focalListPriority{0}, lastEarliestPerturbationSourceId{0}, lastEarliestPerturbationSourceIdCost{cost_t::INFTY} {

        }
        ~GraphFocalState() {

        }
        GraphFocalState(const GraphFocalStateInstance& other) = delete;
        GraphFocalState(GraphFocalStateInstance&& other) : GraphState<G, V, E>{::std::move(other)}, 
            openList{other.openList}, focalList{other.focalList}, focalListPriority{other.focalListPriority}, 
            source{other.source},
            lastEarliestPerturbationSourceId{other.lastEarliestPerturbationSourceId}, 
            lastEarliestPerturbationSourceIdCost{other.lastEarliestPerturbationSourceIdCost} {

        }

        GraphFocalStateInstance& operator =(const GraphFocalStateInstance& other) = delete;
        GraphFocalStateInstance& operator =(GraphFocalStateInstance&& other) {
            GraphState<G, V, E>::operator =(::std::move(other));
            this->openList = other.openList;
            this->focalList = other.focalList;
            this->source = other.source;
            this->focalListPriority = other.focalListPriority;
            this->lastEarliestPerturbationSourceId = other.lastEarliestPerturbationSourceId;
            this->lastEarliestPerturbationSourceIdCost = other.lastEarliestPerturbationSourceIdCost;

            return *this;
        }
    public:
        GraphFocalStateInstance* getParent() {
            return static_cast<GraphFocalStateInstance*>(this->parent);
        }
        const GraphFocalStateInstance* getParent() const {
            return static_cast<const GraphFocalStateInstance*>(this->parent);
        }
        void setParent(GraphFocalStateInstance* parent) {
            this->parent = static_cast<GraphFocalStateInstance*>(parent);
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
        virtual priority_t getPriority(const void* context) const {
            if (context == this->openList.get()) {
                return this->priority;
            } else if (context == this->focalList.get()) {
                return this->focalListPriority;
            } else {
                throw cpp_utils::exceptions::ImpossibleException{"focal is %p open is %p, got %p", this->focalList, this->openList, context};
            }
        }
        virtual void setPriority(const void* context, priority_t p) {
            if (context == this->openList.get()) {
                this->priority = p;
            } else if (context == this->focalList.get()) {
                this->focalListPriority = p;
            } else {
                throw cpp_utils::exceptions::ImpossibleException{"focal is %p open is %p, got %p", this->focalList, this->openList, context};
            }
        }
    };



}

#endif