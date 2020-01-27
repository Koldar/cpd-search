#include "catch.hpp"

#include <pathfinding-utils/GridMap.hpp>
#include <pathfinding-utils/MovingAIGridMapReader.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>

#include "CpdState.hpp"
#include "CpdJumpSearch.hpp"
#include "CpdJumpSearchImageProducerListener.hpp"

using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::search;
using namespace pathfinding::maps;

SCENARIO("test CpdJumpSearch with optimality bound") {
   
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

        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) {return PerturbatedCost{c, false}; })};

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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            start.getPayload();
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{0,1})
            ));
            REQUIRE(solution->getCost() == 100);
        }

        WHEN("start to goal via perturbated edge") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(solution->getCost() == 400);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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

SCENARIO("test CpdJumpSearch with suboptimality bound") {
   
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

        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) {return PerturbatedCost{c, false}; })};

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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            start.getPayload();
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{0,1})
            ));
            REQUIRE(solution->getCost() == 100);
        }

        WHEN("start to goal via perturbated edge") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(solution->getCost() == 400);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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
            CpdState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            CpdState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const CpdState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
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