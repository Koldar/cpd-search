#include "catch.hpp"

#include <pathfinding-utils/GridMap.hpp>
#include <pathfinding-utils/MovingAIGridMapReader.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>

#include "CpdFocalSearch.hpp"

using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::maps;
using namespace pathfinding::search;

SCENARIO("test CpdFocalSearch with optimality bound") {
   
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

        cpp_utils::function_t<GraphFocalState<std::string, xyLoc, PerturbatedCost>, std::tuple<xyLoc>> mapper = [&](auto& x) { return x.getPayload();};

        // USE THE FACTORY TO PROVIDE CpdSearch

        CpdFocalSearchFactory factory{};
        auto factory_output = factory.get(cpdManager, perturbatedGraph, cost_t{1}, fractional_number<cost_t>{cost_t{1}});

        REQUIRE(g.haveSameVertices(perturbatedGraph));

        WHEN("start == goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            auto& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
            start.getPayload();
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(
                solution->map(mapper) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0})
            ));
            REQUIRE(solution->getCost() == 0);
        }

        WHEN("start is diagonal goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{1,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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

SCENARIO("test CpdFocalSearch with suboptimality bound") {
   
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

        CpdFocalSearchFactory factory{};
        //focal bound set to 2 ==> WA*
        auto factory_output = factory.get(cpdManager, perturbatedGraph, fractional_number<cost_t>{2}, fractional_number<cost_t>{1});

        REQUIRE(g.haveSameVertices(perturbatedGraph));

        WHEN("start == goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            auto& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            auto& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, cpd_search_generated_e::INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, cpd_search_generated_e::INPUT);
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