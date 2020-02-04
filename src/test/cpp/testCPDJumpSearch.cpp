#include "catch.hpp"

#include <cpp-utils/functional.hpp>

#include <pathfinding-utils/GridMap.hpp>
#include <pathfinding-utils/MovingAIGridMapReader.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>
#include <pathfinding-utils/pathValidators.hpp>

#include "CpdState.hpp"
#include "CpdJumpSearch.hpp"
#include "CpdJumpSearchImageProducerListener.hpp"
#include "CpdJumpSearchVideoProducerListener.hpp"

using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::validator;
using namespace pathfinding::search;
using namespace pathfinding::search::listeners;
using namespace pathfinding::maps;

SCENARIO("test CpdJumpSearch with optimality bound", "[cpd-jump-search-optimal]") {
   
    GIVEN("a gridmap") {

        // CREATE GRAPH WHERE WE WANT TO OPERATE

        MovingAIGridMapReader reader{
            '.', 100, color_t::WHITE,
            'T', 150, color_t::GREEN, 
            '@', cost_t::INFTY, color_t::BLACK
        };
        GridMap gridMap = reader.load(boost::filesystem::path{"square03.map"});
        GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
        AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(gridMap)};

        /*
         *  01234
         * 0.XX..
         * 1...@@
         * 2..@@.
         * 3...@@
         * 4..XX.
         * 
         * top XX = 150
         * bottom XX=200
         */

        // INCLUDE THE CPD

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./square03.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();

        // CREATE THE TIME GRAPH WITH PERTURBATIONS

        auto perturbatedGraphTmp = std::unique_ptr<IImmutableGraph<std::string, xyLoc, PerturbatedCost>>(cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) { return PerturbatedCost{c, false}; }));
        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*perturbatedGraphTmp};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 0}), perturbatedGraph.idOfVertex(xyLoc{2, 0}), PerturbatedCost{150, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{2, 4}), perturbatedGraph.idOfVertex(xyLoc{3, 4}), PerturbatedCost{200, true});

        // USE THE FACTORY TO PROVIDE CpdSearch

        CpdJumpSearchFactory factory{};
        auto factory_output = factory.get(cpdManager, perturbatedGraph, cost_t{1});

        REQUIRE(g.haveSameVertices(perturbatedGraph));

        WHEN("start == goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            start.getPayload();
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0})
            ));
            REQUIRE(solution->getCost() == 0);
        }

        WHEN("start is diagonal goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{1,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{1,1})
            ));
            REQUIRE(solution->getCost() == 141);
        }

        WHEN("start is above goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{0,1})
            ));
            REQUIRE(solution->getCost() == 100);
        }

        WHEN("start to goal via perturbated edge") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{1,0}),
                    std::make_tuple(xyLoc{2,0}),
                    std::make_tuple(xyLoc{3,0}),
                    std::make_tuple(xyLoc{4,0})
            ));
            REQUIRE(solution->getCost() == 450);
        }

        WHEN("early termination (no perturbation)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,4};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(solution->getCost() == 400);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{0,1}),
                    std::make_tuple(xyLoc{0,2}),
                    std::make_tuple(xyLoc{0,3}),
                    std::make_tuple(xyLoc{0,4})
            ));
        }

        WHEN("early termination (perturbation near the end)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{0,1}),
                    std::make_tuple(xyLoc{0,2}),
                    std::make_tuple(xyLoc{1,3}),
                    std::make_tuple(xyLoc{2,4}),
                    std::make_tuple(xyLoc{3,4}),
                    std::make_tuple(xyLoc{4,4})
            ));
            REQUIRE(solution->getCost() == (2*141 + 3*100 + 1*200));
        }

        WHEN("early termination (perturbation near the start)") {
            xyLoc startLoc{4,4};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{4,4}),
                    std::make_tuple(xyLoc{3,4}),
                    std::make_tuple(xyLoc{2,4}),
                    std::make_tuple(xyLoc{1,3}),
                    std::make_tuple(xyLoc{0,2}),
                    std::make_tuple(xyLoc{0,1}),
                    std::make_tuple(xyLoc{0,0}) 
            ));
            REQUIRE(solution->getCost() == (2*141 + 3*100 + 1*200));
        }

        delete factory_output;
    }
}

SCENARIO("test CpdJumpSearch with suboptimality bound", "[cpd-jump-search-suboptimal]") {
   
    GIVEN("a gridmap") {

        // CREATE GRAPH WHERE WE WANT TO OPERATE

        MovingAIGridMapReader reader{
            '.', 100, color_t::WHITE,
            'T', 150, color_t::GREEN, 
            '@', cost_t::INFTY, color_t::BLACK
        };
        GridMap gridMap = reader.load(boost::filesystem::path{"square03.map"});
        GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
        AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(gridMap)};

        /*
         *  01234
         * 0.XX..
         * 1...@@
         * 2..@@.
         * 3...@@
         * 4..XX.
         * 
         * top XX = 150
         * bottom XX=200
         */

        // INCLUDE THE CPD

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./square03.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();

        // CREATE THE TIME GRAPH WITH PERTURBATIONS

        auto perturbatedGraphTmp = std::unique_ptr<IImmutableGraph<std::string, xyLoc, PerturbatedCost>>(cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) { return PerturbatedCost{c, false}; }));
        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*perturbatedGraphTmp};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 0}), perturbatedGraph.idOfVertex(xyLoc{2, 0}), PerturbatedCost{150, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{2, 4}), perturbatedGraph.idOfVertex(xyLoc{3, 4}), PerturbatedCost{200, true});

        // USE THE FACTORY TO PROVIDE CpdSearch

        CpdJumpSearchFactory factory{};
        //focal bound set to 2 ==> WA*
        auto factory_output = factory.get(cpdManager, perturbatedGraph, fractional_number<cost_t>{2});

        REQUIRE(g.haveSameVertices(perturbatedGraph));

        WHEN("start == goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            start.getPayload();
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0})
            ));
            REQUIRE(solution->getCost() == 0);
        }

        WHEN("start is diagonal goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{1,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{1,1})
            ));
            REQUIRE(solution->getCost() == 141);
        }

        WHEN("start is above goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{0,1})
            ));
            REQUIRE(solution->getCost() == 100);
        }

        WHEN("start to goal via perturbated edge") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{1,0}),
                    std::make_tuple(xyLoc{2,0}),
                    std::make_tuple(xyLoc{3,0}),
                    std::make_tuple(xyLoc{4,0})
            ));
            REQUIRE(solution->getCost() == 450);
        }

        WHEN("early termination (no perturbation)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,4};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(solution->getCost() == 400);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{0,1}),
                    std::make_tuple(xyLoc{0,2}),
                    std::make_tuple(xyLoc{0,3}),
                    std::make_tuple(xyLoc{0,4})
            ));
        }

        WHEN("early termination (perturbation near the end)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{0,1}),
                    std::make_tuple(xyLoc{0,2}),
                    std::make_tuple(xyLoc{1,3}),
                    std::make_tuple(xyLoc{2,4}),
                    std::make_tuple(xyLoc{3,4}),
                    std::make_tuple(xyLoc{4,4})
            ));
            REQUIRE(solution->getCost() == (2*141 + 3*100 + 1*200));
        }

        WHEN("early termination (perturbation near the start)") {
            xyLoc startLoc{4,4};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId);
            auto& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](auto& x) {
                    return x.getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{4,4}),
                    std::make_tuple(xyLoc{3,4}),
                    std::make_tuple(xyLoc{2,4}),
                    std::make_tuple(xyLoc{1,3}),
                    std::make_tuple(xyLoc{0,2}),
                    std::make_tuple(xyLoc{0,1}),
                    std::make_tuple(xyLoc{0,0}) 
            ));
            REQUIRE(solution->getCost() == (2*141 + 3*100 + 1*200));
        }

        delete factory_output;
    }
}

template <typename G, typename V, typename E>
void performCpdJumpSearchOptimalTest(xyLoc startLoc, xyLoc goalLoc, const GridMap& gridMap, const boost::filesystem::path& basename, const IImmutableGraph<G,V,E>& originalMap, const IImmutableGraph<G,V,PerturbatedCost>& perturbatedGraph, const CpdManager<G,V>& cpdManager, fractional_cost epsilon) {
    static costFunction_t<PerturbatedCost> costFunction = [&](auto pc) { return pc.getCost();};
    CpdJumpSearchFactory factory{};
    //focal bound set to 2 ==> WA*
    auto factory_output = factory.get(cpdManager, perturbatedGraph, epsilon);
    CpdJumpSearchVideoProducerListener<std::string, xyLoc, cost_t, PerturbatedCost, CpdState<std::string, xyLoc, PerturbatedCost, cpd_search_generated_e>, GridMap, GridMapImage> listener{originalMap, perturbatedGraph, gridMap, costFunction};
    factory_output->search.setListener(listener);

    nodeid_t startId = perturbatedGraph.idOfVertex(startLoc);
    nodeid_t goalId = perturbatedGraph.idOfVertex(goalLoc);

    factory_output->search.setupSearch(nullptr, nullptr);
    auto start = &(factory_output->stateSupplier.getState(startId));
    auto goal = &(factory_output->stateSupplier.getState(goalId));

    auto solution = factory_output->search.search(*start, *goal, false, false);
    critical("solution found!");

    info("drawing image of the perturbated graph together with search info!");
    // //draw the grid and the perturbations on it
    // GridMapImage* image = gridMap.getPPM();
    // *image = static_cast<GridMapImage&>(addPerturbationsOnMap(
    //     *image, gridMap, 
    //     originalMap, perturbatedGraph,
    //     color_t::RED, color_t::BLUE, costFunction
    // ));
    // function_t<VertexInfo<xyLoc>, statevisited_e> toState = [&](auto e) { return e.state;};
    // *image = static_cast<GridMapImage&>(addExpandedNodesInImage(
    //     *image, gridMap,
    //     perturbatedGraph, listener.getVisitedStates(), toState,
    //     color_t::BLACK, color_t::YELLOW.scale(0.6), color_t::YELLOW.scale(0.8)
    // ));
    
    // *image = static_cast<GridMapImage&>(addPathInImage(
    //     *image, gridMap, 
    //     perturbatedGraph, listener.getSolution(),
    //     color_t::VIOLET.scale(0.6), color_t::VIOLET.scale(0.3), color_t::VIOLET.scale(0.3)
    // ));

    // image->saveBMP("./query1");
    // delete image;

    // cpp_utils::function_t<
    //     pathfinding::search::CpdState<
    //         std::__cxx11::basic_string<char>, 
    //         pathfinding::xyLoc, 
    //         pathfinding::search::PerturbatedCost, 
    //         pathfinding::cpd_search_generated_e
    //     >, 
    //     long unsigned int
    // >
        
    // std::function<
    //     long unsigned int(
    //         const pathfinding::search::CpdState<
    //             std::__cxx11::basic_string<char>, 
    //             pathfinding::xyLoc, 
    //             pathfinding::search::PerturbatedCost, 
    //             pathfinding::cpd_search_generated_e
    //         >&)
    //     >

    cpp_utils::function_t<CpdState<G, V, PerturbatedCost, cpd_search_generated_e>, nodeid_t> mapper2 = [&](auto& x) { return x.getPosition(); };

    validator::checkIfPathSuboptimalityBound(
        2.0,
        perturbatedGraph,
        startId, goalId,
        *solution,
        costFunction,
        mapper2
    );
}


SCENARIO("test cpd focal optimal search failed queries", "[cpd-jump-search-queries]") {

    //CREATE ORIGINAL GRAPH
    MovingAIGridMapReader reader{
        '.', 1000, color_t::WHITE,
        'T', 1500, color_t::GREEN,
        'S', 2000, color_t::CYAN,
        'W', 2500, color_t::BLUE,
        '@', cost_t::INFTY, color_t::BLACK
    };

    // GIVEN("query 1") {

    //     GridMap gridMap = reader.load(boost::filesystem::path{"./arena2.map"});
    //     GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
    //     AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(gridMap)};
    //     // INCLUDE THE CPD REORDERING
    //     CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./arena2.cpd"}, graph};
    //     const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();
    //     // CREATE PERTURBATED GRAPH
    //     boost::filesystem::path perturbatedEdgesPath{"./arena2-900-perturbated.graph"};
    //     cpp_utils::function_t<cost_t, PerturbatedCost> mapper = [&](const cost_t& c) { return PerturbatedCost{c, false};};
    //     std::unique_ptr<IImmutableGraph<std::string,xyLoc,PerturbatedCost>> perturbatedGraph{pathfinding::utils::loadPerturbatedMap(
    //             perturbatedEdgesPath, g, mapper
    //     )};
    //     // USE THE FACTORY TO PROVIDE THE SEARCH
    //     REQUIRE(g.haveSameVertices(*perturbatedGraph));

    //     WHEN("query 1 with epsilon set to 2") {
    //         xyLoc startLoc{16, 105};
    //         xyLoc goalLoc{277, 206};
    //         performCpdJumpSearchOptimalTest(
    //             startLoc, goalLoc, 
    //             gridMap, "./query1", 
    //             g, *perturbatedGraph, 
    //             cpdManager, 
    //             fractional_cost{2}
    //         );
    //     }
    // }

    GIVEN("query 1") {

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

        WHEN("query 1 with epsilon set to 1.1") {
            xyLoc startLoc{16, 105};
            xyLoc goalLoc{277, 206};
            performCpdJumpSearchOptimalTest(
                startLoc, goalLoc, 
                gridMap, "./query1", 
                g, *perturbatedGraph, 
                cpdManager, 
                fractional_cost{1.1}
            );
        }
    }
}