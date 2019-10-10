#ifndef _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__
#define _CPD_SEARCH_GRAPH_FOCAL_STATE_HEADER__

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
    class GraphFocalState: GraphState<G, V, E> {
        typedef GraphFocalState<G, V, E> GraphFocalStateInstance;
    private:
        /**
         * @brief reference to an openlist
         * 
         */
        IQueue<GraphFocalState>* openList;
        /**
         * @brief reference to a focal list
         * 
         */
        IQueue<GraphFocalState>* focalList;
        /**
         * @brief priority of the node in the focal list
         * 
         */
        priority_t focalListPriority;
    public:
        GraphFocalState(stateid_t id, const IImmutableGraph<G, V, E>& g, nodeid_t location): GraphState<G, V, E>{id, g, location}, openList{nullptr}, focalList{nullptr}, focalListPriority{0} {

        }
        ~GraphFocalState() {

        }
        GraphFocalState(const GraphFocalStateInstance& other) = delete;
        GraphFocalState(GraphFocalStateInstance&& other) : GraphState<G, V, E>{::std::move(other)}, openList{other.openList}, focalList{other.focalList}, focalListPriority{other.focalListPriority} {
            other.openList = nullptr;
            other.focalList = nullptr,
        }

        GraphFocalStateInstance& operator =(const GraphFocalStateInstance& other) = delete;
        GraphFocalStateInstance& operator =(GraphFocalStateInstance&& other) {
            Graph<G, V, E>::operator =(::std::move(other));
            this->openList = other.openList;
            this->focalList = other.focalList;
            this->focalListPriority = other.focalListPriority;

            other.openList = nullptr;
            other.focalList = nullptr;
            return *this;
        }
    public:
        virtual priority_t getPriority(const void* context) const {
            if (context == this->openList) {
                return this->priority;
            } else if (context == this->focalList) {
                return this->focalListPriority;
            } else {
                throw cpp_utils::exceptions::ImpossibleException{};
            }
        }
        virtual void setPriority(const void* context, priority_t p) {
            if (context == this->openList) {
                this->priority = p;
            } else if (context == this->focalList) {
                this->focalListPriority = p;
            } else {
                throw cpp_utils::exceptions::ImpossibleException{};
            }
        }
    };

}

#endif