#include "catch.hpp"

#include <pathfinding-utils/MovingAIGridMapReader.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>
#include <pathfinding-utils/utils.hpp>
#include <pathfinding-utils/pathValidators.hpp>

#include "CpdSearchTrackerListener.hpp"
#include "CpdFocalSearchTrackerListener.hpp"
#include "CpdFocalOptimalSearch.hpp"
#include "PerturbatedCost.hpp"
#include "CpdFocalSearchImageProducerListener.hpp"

using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::utils;
using namespace pathfinding::maps;
using namespace pathfinding::search;
using namespace pathfinding::search::listeners;


SCENARIO("test CpdFocalOptimalSearch with suboptimality bound", "[cpd-focal-optimal-search]") {
   
    GIVEN("a gridmap") {

        // CREATE GRAPH WHERE WE WANT TO OPERATE

        MovingAIGridMapReader reader{
            '.', 100, color_t::WHITE,
            'T', 150, color_t::GREEN, 
            '@', cost_t::INFTY, color_t::BLACK
        };
        GridMap gridMap = reader.load(boost::filesystem::path{"square04.map"});
        GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
        AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(gridMap)};

        /*
         *  01234
         * 0.....
         * 1...@@
         * 2...@.
         * 3.X..@
         * 4.X...
         * 
         * top X = 250
         */

        // INCLUDE THE CPD

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./square04.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();

        // CREATE THE TIME GRAPH WITH PERTURBATIONS

        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) {return PerturbatedCost{c, false}; })};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{0, 4}), perturbatedGraph.idOfVertex(xyLoc{1, 4}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{0, 4}), perturbatedGraph.idOfVertex(xyLoc{1, 3}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{0, 3}), perturbatedGraph.idOfVertex(xyLoc{1, 3}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{0, 3}), perturbatedGraph.idOfVertex(xyLoc{1, 4}), PerturbatedCost{250, true});

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 4}), perturbatedGraph.idOfVertex(xyLoc{2, 4}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 4}), perturbatedGraph.idOfVertex(xyLoc{2, 3}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 3}), perturbatedGraph.idOfVertex(xyLoc{2, 3}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 3}), perturbatedGraph.idOfVertex(xyLoc{2, 4}), PerturbatedCost{250, true});

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{0, 2}), perturbatedGraph.idOfVertex(xyLoc{1, 3}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 2}), perturbatedGraph.idOfVertex(xyLoc{1, 3}), PerturbatedCost{250, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{2, 2}), perturbatedGraph.idOfVertex(xyLoc{1, 3}), PerturbatedCost{250, true});

        // USE THE FACTORY TO PROVIDE CpdSearch

        CpdFocalOptimalSearchFactory factory{};
        //focal bound set to 2 ==> WA*
        auto factory_output = factory.get(cpdManager, perturbatedGraph, fractional_number<cost_t>{2}, fractional_number<cost_t>{1});

        REQUIRE(g.haveSameVertices(perturbatedGraph));

        WHEN("start == goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            start.getPayload();
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0})
            ));
            REQUIRE(solution->getCost() == 0);
        }

        WHEN("start is diagonal goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{1,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{1,1})
            ));
            REQUIRE(solution->getCost() == 141);
        }

        WHEN("start is above goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{0,1})
            ));
            REQUIRE(solution->getCost() == 100);
        }

        WHEN("start to goal via perturbated edge") {

            /*
            *  01234
            * 0.....
            * 1...@@
            * 2...@.
            * 3.X..@
            * 4.X...
            * 
            * top X = 250
            */

            xyLoc startLoc{4,4}; 
            xyLoc goalLoc{0,4};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            critical("fetch a solution with cost of", solution->getCost());
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{4,4}), 
                    std::make_tuple(xyLoc{3,4}),
                    std::make_tuple(xyLoc{2,3}),
                    std::make_tuple(xyLoc{1,2}),
                    std::make_tuple(xyLoc{0,3}),
                    std::make_tuple(xyLoc{0,4})
            ));
            REQUIRE(solution->getCost() == (2*100 + 3*141));
        }

        delete factory_output;
    }
}

template <typename G, typename V, typename E>
void performCpdFocalSearchOptimalTest(xyLoc startLoc, xyLoc goalLoc, const GridMap& gridMap, const boost::filesystem::path& basename, const IImmutableGraph<G,V,E>& originalMap, const IImmutableGraph<G,V,PerturbatedCost>& perturbatedGraph, const CpdManager<G,V>& cpdManager, fractional_cost w, fractional_cost epsilon) {
    static costFunction_t<PerturbatedCost> costFunction = [&](auto pc) { return pc.getCost();};
    CpdFocalOptimalSearchFactory factory{};
    //focal bound set to 2 ==> WA*
    auto factory_output = factory.get(cpdManager, perturbatedGraph, w, epsilon);
    CpdFocalSearchImageProducerListener<std::string, xyLoc, cost_t, PerturbatedCost, GraphFocalState<std::string, xyLoc, PerturbatedCost>, GridMap, GridMapImage> listener{originalMap, perturbatedGraph, gridMap, costFunction};
    //CpdFocalSearchTrackerListener<std::string, xyLoc, PerturbatedCost, GraphFocalState<std::string, xyLoc, PerturbatedCost>> listener{perturbatedGraph};
    factory_output->search.setListener(listener);

    nodeid_t startId = perturbatedGraph.idOfVertex(startLoc);
    nodeid_t goalId = perturbatedGraph.idOfVertex(goalLoc);

    factory_output->search.setupSearch(nullptr, nullptr);
    auto start = &(factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT));
    auto goal = &(factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT));

    auto solution = factory_output->search.search(*start, *goal, false, false);
    critical("solution found!");

    info("drawing image of the perturbated graph together with search info!");
    //draw the grid and the perturbations on it
    GridMapImage* image = gridMap.getPPM();
    *image = static_cast<GridMapImage&>(addPerturbationsOnMap(
        *image, gridMap, 
        originalMap, perturbatedGraph,
        color_t::RED, color_t::BLUE, costFunction
    ));
    function_t<VertexInfo<xyLoc>, statevisited_e> toState = [&](auto e) { return e.state;};
    *image = static_cast<GridMapImage&>(addExpandedNodesInImage(
        *image, gridMap,
        perturbatedGraph, listener.getVisitedStates(), toState,
        color_t::BLACK, color_t::YELLOW.scale(0.6), color_t::YELLOW.scale(0.8)
    ));
    
    *image = static_cast<GridMapImage&>(addPathInImage(
        *image, gridMap, 
        perturbatedGraph, listener.getSolution(),
        color_t::VIOLET.scale(0.6), color_t::VIOLET.scale(0.3), color_t::VIOLET.scale(0.3)
    ));

    image->saveBMP("./query1");
    delete image;

    std::function<nodeid_t(const GraphFocalState<std::string, xyLoc, PerturbatedCost>*)> mapper2 = [&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) -> nodeid_t { return x->getPosition(); };

    validator::checkIfPathOptimal(
        perturbatedGraph,
        startId, goalId,
        *solution,
        costFunction,
        mapper2
    );
}

SCENARIO("test cpd focal optimal search failed queries") {

    GIVEN("query 1") {

        //CREATE ORIGINAL GRAPH
        MovingAIGridMapReader reader{
            '.', 1000, color_t::WHITE,
            'T', 1500, color_t::GREEN,
            'S', 2000, color_t::CYAN,
            'W', 2500, color_t::BLUE,
            '@', cost_t::INFTY, color_t::BLACK
        };
        GridMap gridMap = reader.load(boost::filesystem::path{"./arena2.map"});
        GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
        AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(gridMap)};
        // INCLUDE THE CPD REORDERING
        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./arena2.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();
        // CREATE PERTURBATED GRAPH
        boost::filesystem::path perturbatedEdgesPath{"./arena2-900-perturbated.graph"};
        cpp_utils::function_t<cost_t, PerturbatedCost> mapper = [&](const cost_t& c) { return PerturbatedCost{c, false};};
        std::unique_ptr<IImmutableGraph<std::string,xyLoc,PerturbatedCost>> perturbatedGraph{pathfinding::utils::loadPerturbatedMap(
                perturbatedEdgesPath, g, mapper
        )};
        // USE THE FACTORY TO PROVIDE THE SEARCH
        REQUIRE(g.haveSameVertices(*perturbatedGraph));

        WHEN("query 1") {
            xyLoc startLoc{16, 105};
            xyLoc goalLoc{277, 206};
            performCpdFocalSearchOptimalTest(
                startLoc, goalLoc, 
                gridMap, "./query1", 
                g, *perturbatedGraph, 
                cpdManager, 
                fractional_cost{1.1, 1e-3}, fractional_cost{1}
            );
        }

    }
}