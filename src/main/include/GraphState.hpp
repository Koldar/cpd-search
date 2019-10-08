// /**
//  * @file TimeGridMapState.hpp
//  * @author Massimo Bono
//  * @brief definition of every involving the a state of a gridmap which search is time dependent
//  * @version 0.1
//  * @date 2019-09-16
//  * 
//  * @copyright Copyright (c) 2019
//  * 
//  */
// #ifndef _CPD_SEARCH_GRIDMAPSTATE_HEADER__
// #define _CPD_SEARCH_GRIDMAPSTATE_HEADER__

// #include <pathfinding-utils/ISearchState.hpp>
// #include <pathfinding-utils/IStateSupplier.hpp>
// #include <pathfinding-utils/IStateExpander.hpp>
// #include <pathfinding-utils/IGoalChecker.hpp>
// #include <cpp-utils/StaticPriorityQueue.hpp>
// #include <cpp-utils/igraph.hpp>
// #include <cpp-utils/mapplus.hpp>
// #include <cpp-utils/vectorplus.hpp>
// #include <cpp-utils/adjacentGraph.hpp>
// #include <tuple>

// namespace pathfinding::search {

//     using namespace pathfinding;
//     using namespace cpp_utils::graphs;

//     /**
//      * @brief A* state where a state is identified by a location xyLoc in a grid and the timestamp when the agent reaches such location
//      * 
//      */
//     template <typename G, typename V>
//     class GraphState: public IAstarState<GraphState<G, V>>, cpp_utils::HasPriority {
//     private:
//         /**
//          * @brief f value in A*
//          * 
//          */
//         cost_t f;
//         /**
//          * @brief g value in A*
//          * 
//          */
//         cost_t g;
//         /**
//          * @brief h value in A*
//          * 
//          */
//         cost_t h;
//         /**
//          * @brief parent of state. If null the state do not have any parent
//          * 
//          */
//         GraphState<G,V>* parent;
//         /**
//          * @brief id uniquely identifying tyhe state
//          * 
//          */
//         stateid_t id;
//         /**
//          * @brief if true it means this state should belong to the close list of A*
//          * 
//          */
//         bool expanded;
//         /**
//          * @brief field necessary if you want to put this instance in a queue
//          * 
//          */
//         priority_t priority;
//         /**
//          * @brief the graph where the state will be located
//          */
//         const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& graph;
//         /**
//          * @brief a structure containing where we are in the graph representing the map
//          * 
//          */
//         nodeid_t position;
//     public:
//         GraphState(cost_t f, cost_t g, cost_t h, GraphState* parent, stateid_t id, bool expanded, const IImmutableGraph<G, V, cost_t>& graph, nodeid_t position): f{f}, g{g}, h{h}, parent{parent}, id{id}, expanded{expanded}, graph{graph}, position{position}, priority{0} {

//         }

//         GraphState(stateid_t id, const IImmutableGraph<G, V, cost_t>& graph, nodeid_t position): f{0}, g{0}, h{0}, parent{nullptr}, id{id}, expanded{false}, graph{graph}, position{position}, priority{0} {

//         }

//         GraphState(const GraphState<G,V>& other): f{other.f}, g{other.g}, h{other.h}, parent{other.parent}, id{other.id}, expanded{other.expanded}, graph{other.graph}, position{other.position}, priority{other.priority} {
//         }
//         GraphState<G, V>& operator =(const GraphState<G,V>& other) {
//             assert(this->graph == other.graph);
//             this->f = other.f;
//             this->g = other.g;
//             this->h = other.h;
//             this->parent = other.parent;
//             this->id = other.id;
//             this->expanded = other.expanded;
//             this->position = other.position;
//             this->priority = other.priority;
//             return *this;
//         }
        
//         //TODO we should create a IState which is based upon a graph (in pathfinding utils)
//         nodeid_t getPosition() const {
//             return this->position;
//         }
        
//         /**
//          * @brief get a tuple containing a set of values which allows you to human identify the state
//          * 
//          * @return a tuple containing n the first index the node id in the graph ajnd the second value is the timestamp
//          */
//         std::tuple<V> getValues() const {
//             return std::tuple<V>{this->graph.getVertex(this->getPosition())};
//         }
//     public:
//         friend bool operator <(const GraphState<G,V>& a, const GraphState<G,V>& b) {
//             if(a.f < b.f) {
//                 return true;
//             } else if (b.f < a.f) {
//                 return false;
//             }
//             //ok, it appeans that a and b have the same f. We need some tie breaking mechanism
//             // break ties in favour of larger g
//             if(a.g > b.g) {
//                 return true;
//             }
//             return false;
//         }
//         friend bool operator ==(const GraphState<G,V>& a, const GraphState<G,V>& b) {
//             if (&a == &b) {
//                 return true;
//             }
//             if (a.getId() != b.getId()) {
//                 return false;
//             }
//             return a.getPosition() == b.getPosition();
//         }
//         friend std::ostream& operator <<(std::ostream& out, const GraphState<G,V>& state) {
//             out 
//                 << "{id: " 
//                 << state.getId() 
//                 << " node: " 
//                 << state.graph.getVertex(state.getPosition())
//                 << "}";
//             return out;
//         }
//     public:
//         priority_t getPriority() const {
//             return this->priority;
//         }
//         void setPriority(priority_t p) {
//             this->priority = p;
//         }
//     public:
//         virtual void setF(cost_t f) {
//             this->f = f;
//         }
//         virtual void setG(cost_t g) {
//             this->g = g;
//         }
//         virtual void setH(cost_t h) {
//             this->h = h;
//         }
//         virtual void setParent(GraphState<G,V>* parent) {
//         this->parent = parent;
//         }
//         virtual void setId(stateid_t id) {
//             this->id = id;
//         }
//         virtual void setExpanded(bool expanded) {
//             this->expanded = expanded;
//         }
//         virtual cost_t getF() const {
//             return this->f;
//         }
//         virtual cost_t getG() const {
//             return this->g;
//         }
//         virtual cost_t getH() const {
//             return this->h;
//         }
//         virtual GraphState<G,V>* getParent() {
//             return this->parent;
//         }
//         virtual const GraphState<G,V>* getParent() const {
//             return this->parent;
//         }
//         virtual stateid_t getId() const {
//         return this->id;
//         }
//         virtual bool isExpanded() const {
//             return this->expanded;
//         }
//     public:
//         MemoryConsumption getByteMemoryOccupied() const {
//             return sizeof(*this);
//         }
//     };

//     template <typename G, typename V>
//     class GraphStateSupplier: public IStateSupplier<GraphState<G,V>, nodeid_t> {
//     private:
//         /**
//          * @brief heap memory where the states are concretely saved
//          * 
//          */
//         cpp_utils::vectorplus<GraphState<G,V>> statePool;
//         /**
//          * @brief id that the enxt state which we will create will have
//          * 
//          */
//         stateid_t nextState;
//         const IImmutableGraph<G, V, cost_t>& graph;
//     public:
//         /**
//          * @brief Construct a new Graph State Supplier object
//          * 
//          * @param graph the graph that will be put in ever state generated in this structure
//          */
//         GraphStateSupplier(const IImmutableGraph<G, V, cost_t>& graph): graph{graph}, statePool{10000, GraphState<G,V>{0, graph, 0}}, nextState{0} {

//         }
//         ~GraphStateSupplier() {
//             this->cleanup();
//         }
//         GraphStateSupplier(const GraphStateSupplier<G, V>& other) = delete;
//         GraphStateSupplier(GraphStateSupplier<G, V>&& other) : statePool{::std::move(other.statePool)}, nextState{other.nextState}, graph{other.graph} {

//         }
//         GraphStateSupplier& operator = (const GraphStateSupplier<G, V>& other) = delete;
//         GraphStateSupplier& operator = (GraphStateSupplier<G, V>&& other) {
//             this->statePool = ::std::move(other.statePool);
//             this->nextState = other.nextState;
//             this->graph = other.graph;
//             return *this;
//         }
//     public:
//         virtual GraphState<G, V>& getState(stateid_t id, nodeid_t location) {
//             throw cpp_utils::exceptions::ImpossibleException{"not to implement!"};
//         }
//         virtual GraphState<G, V>& getState(nodeid_t location) {
//             info("location is ", location);
//             //let's check if we have the requested timestamp in fromTimestampToLocationMap
//             if (this->statePool[location] == nullptr) {
//                 stateid_t id = nextState;
//                 nextState += 1;
//                 this->statePool[id] = GraphState<G, V>{id, graph, location};
//             }
//             return this->statePool[location];
//         }
//     public:
//         virtual void cleanup() {
//             this->nextState = 0;
//             this->statePool.cleanup();
//         }
//     public:
//         virtual MemoryConsumption getByteMemoryOccupied() const {
//             throw cpp_utils::exceptions::NotYetImplementedException{""};
//         }
//     private:

//     };

//     /**
//      * @brief allows to generate the successors of a GridMapState
//      * 
//      * @tparam G payload of the underlying weighted directed graph
//      */
//     template <typename G, typename V>
//     class GraphStateExpander: public IStateExpander<GraphState<G,V>, nodeid_t> {
//     private:
//         const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& graph;
//     public:
//         GraphStateExpander(const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& graph): graph{graph} {

//         }
//         virtual ~GraphStateExpander() {

//         }
//         GraphStateExpander(const GraphStateExpander<G, V>& other) : graph{other.graph} {

//         }
//         GraphStateExpander<G, V>& operator =(const GraphStateExpander<G, V>& other) = delete;
//         GraphStateExpander(GraphStateExpander<G, V>&& other) : graph{other.graph} {

//         }
//         GraphStateExpander<G, V>& operator =(GraphStateExpander<G, V>&& other) {
//             this->graph = other.graph;
//             return *this;
//         }
//     public:
//         virtual cpp_utils::vectorplus<std::pair<GraphState<G,V>&, cost_t>> getSuccessors(const GraphState<G,V>& state, IStateSupplier<GraphState<G,V>, nodeid_t>& supplier) {
//             cpp_utils::vectorplus<std::pair<GraphState<G,V>&, cost_t>> result{};
//             //****************** MOVING *********************
//             for (auto outEdge : graph.getOutEdges(state.getPosition())) {
//                 fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
//                 result.add(std::pair<GraphState<G,V>&, cost_t>{
//                     supplier.getState(outEdge.getSinkId()),
//                     outEdge.getPayload()
//                 });
//             }

//             return result;
//         }
//     public:
//         virtual void cleanup() {

//         }
//     public:
//         virtual MemoryConsumption getByteMemoryOccupied() const {
//             return sizeof(*this);
//         }
//     };

//     /**
//      * @brief true if the state position is the same as the goal one
//      * 
//      */
//     template <typename G, typename V>
//     class GraphStateGoalChecker: public IGoalChecker<GraphState<G, V>> {
//     public:
//         virtual bool isGoal(const GraphState<G,V>& state, const GraphState<G,V>* goal) const {
//             return state.getPosition() == goal->getPosition();
//         }
//     public:
//         virtual void cleanup() {

//         }
//     public:
//         virtual MemoryConsumption getByteMemoryOccupied() const {
//             return sizeof(*this);
//         }
// };



// }


// #endif