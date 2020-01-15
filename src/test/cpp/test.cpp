#include "catch.hpp"

#include <boost/filesystem.hpp>

#include <cpp-utils/adjacentGraph.hpp>
#include <pathfinding-utils/types.hpp>

#include <pathfinding-utils/GridMap.hpp>
#include <pathfinding-utils/MovingAIGridMapReader.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>

#include <compressed-path-database/CpdManager.hpp>


#include "CpdHeuristic.hpp"
#include "CpdSearch.hpp"
#include "CpdHeuristic.hpp"
#include "CpdFocalSearch.hpp"
#include "CountCpdSearchListener.hpp"

using namespace pathfinding::search;
using namespace pathfinding::maps;
using namespace cpp_utils::graphs;
using namespace compressed_path_database;


SCENARIO("test CpdHeuristic") {

    GIVEN("a gridmap") {

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
         * 0.....
         * 1...@@
         * 2..@@.
         * 3...@@
         * 4.....
         * 
         */

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./square03.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& reorderedGraph = cpdManager.getReorderedGraph();
        // IT'S REALLY IMPORTANT THAT WE QUERY NOT ON "graph" BUT ON "reorderedGraph"!!!!!
        // THIS because the CPD uses those indices, not the ones from "graph"!!!
        std::function<PerturbatedCost(const cost_t&)> costFunction = [&](const cost_t& c) { return PerturbatedCost{c, false};};
        std::unique_ptr<IImmutableGraph<std::string, xyLoc, PerturbatedCost>> perturbatedGraph = reorderedGraph.mapEdges(costFunction);
        CpdHeuristic<GraphState<std::string, xyLoc>, std::string, xyLoc> h{
            cpdManager, 
            *perturbatedGraph
        };

        REQUIRE(h.isAdmissible());
        REQUIRE(h.isConsistent());

        WHEN("start=goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 0);
        }

        WHEN("start above goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,1};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 100);
            REQUIRE(h.getCachedElementsNumber() == 2);
        }

        WHEN("start diagonal goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{1,1};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 141);
            REQUIRE(h.getCachedElementsNumber() == 2);
        }

        WHEN("start distance from goal of 2") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{2,0};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 200);
            REQUIRE(h.getCachedElementsNumber() == 3);
        }

        WHEN("start diangonal distance from goal of 2") {
            xyLoc startLoc{0,2};
            xyLoc goalLoc{2,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 282);
            REQUIRE(h.getCachedElementsNumber() == 3);
        }

        WHEN("start far from goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == (100*4 + 141*2));
            REQUIRE(h.getCachedElementsNumber() == 7);
        }

        WHEN("reusing heuristic and paths are not overlapping (except goal") {
            xyLoc startLoc1{0,0};
            xyLoc startLoc2{4,0};
            xyLoc goalLoc{2,0};
            GraphState<std::string, xyLoc> start1{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc1)};
            GraphState<std::string, xyLoc> start2{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc2)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start1, &goal) == (100*2));
            REQUIRE(h.getHeuristic(start2, &goal) == (100*2));

            REQUIRE(h.getCachedElementsNumber() == 5);
        }

        //TODO coniotnue
        WHEN("reusing heuristic and paths are overlapping in the middle") {
            xyLoc startLoc1{0,0};
            xyLoc startLoc2{2,1};
            xyLoc goalLoc{4,0};
            GraphState<std::string, xyLoc> start1{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc1)};
            GraphState<std::string, xyLoc> start2{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc2)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start1, &goal) == (100*4));
            REQUIRE(h.getHeuristic(start2, &goal) == (100*3));
            REQUIRE(h.getCachedElementsNumber() == 6);
        }

        WHEN("reusing heuristic and one path is subset of the other") {
            xyLoc startLoc1{0,0};
            xyLoc startLoc2{2,0};
            xyLoc goalLoc{4,0};
            GraphState<std::string, xyLoc> start1{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc1)};
            GraphState<std::string, xyLoc> start2{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc2)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start1, &goal) == (100*4));
            REQUIRE(h.getHeuristic(start2, &goal) == (100*2));
            REQUIRE(h.getCachedElementsNumber() == 5);
        }

        WHEN("performing cleanup") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            h.getHeuristic(start, &goal);

            REQUIRE(h.getCachedElementsNumber() == 7);
            h.cleanup();
            REQUIRE(h.getCachedElementsNumber() == 0);
        }

        WHEN("goal unreachable") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,2};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == cost_t::INFTY);
            REQUIRE(h.getCachedElementsNumber() == 0);
        }
    }

}

SCENARIO("test GraphState supporting concepts") {

    GIVEN("a gridmap") {

        MovingAIGridMapReader reader{
            '.', 100, color_t::WHITE,
            'T', 150, color_t::GREEN,
            '@', cost_t::INFTY, color_t::BLACK
        };
        GridMap gridMap = reader.load("square03.map");
        GridMapGraphConverter converter{GridBranching::EIGHT_CONNECTED};
        std::unique_ptr<IImmutableGraph<std::string, xyLoc, cost_t>> graph = converter.toGraph(gridMap);


        /*
         *  01234
         * 0.....
         * 1...@@
         * 2..@@.
         * 3...@@
         * 4.....
         * 
         */

        GraphStateSupplier<std::string, xyLoc, cost_t> supplier{*graph};

        WHEN("test GraphStateSupplier") {
            
            THEN("state not yet generated") {
                GraphState<std::string, xyLoc, cost_t>& s = supplier.getState(graph->idOfVertex(xyLoc{1,3}));
                REQUIRE(s.getPosition() == graph->idOfVertex(xyLoc{1,3}));
                REQUIRE(s.getId() == graph->idOfVertex(xyLoc{1,3}));
            }

            THEN("state already generated") {
                GraphState<std::string, xyLoc>& s = supplier.getState(graph->idOfVertex(xyLoc{1,3}));
                GraphState<std::string, xyLoc>& s2 = supplier.getState(graph->idOfVertex(xyLoc{1,3}));
                REQUIRE(s.getPosition() == graph->idOfVertex(xyLoc{1,3}));
                REQUIRE(s.getId() == graph->idOfVertex(xyLoc{1,3}));
                REQUIRE(s2.getPosition() == graph->idOfVertex(xyLoc{1,3}));
                REQUIRE(s2.getId() == graph->idOfVertex(xyLoc{1,3}));
                REQUIRE(&s == &s2);
            }
        }

        StandardStateExpander<GraphState<std::string, xyLoc, cost_t>, std::string, xyLoc> expander{*graph};

        WHEN("test GraphStateExpander") {
            GraphState<std::string, xyLoc, cost_t> s = supplier.getState(graph->idOfVertex(xyLoc{1,3}));
            cpp_utils::vectorplus<std::pair<GraphState<std::string, xyLoc, cost_t>&, cost_t>> successors = expander.getSuccessors(s, supplier);
            auto states = successors.map<GraphState<std::string, xyLoc>>([&](std::pair<GraphState<std::string, xyLoc>&, cost_t> p) {return p.first; });
            info("successors: ", successors);
            REQUIRE(successors.size() == 7);

            IImmutableGraph<std::string, xyLoc, cost_t>* originalGraph = graph.get();

            //moves
            REQUIRE(successors
                .filter([&, originalGraph](std::pair<GraphState<std::string,xyLoc>, cost_t> p) {
                    return originalGraph->getVertex(p.first.getPosition()) == xyLoc{2,3};
                }).size() == 1);
            REQUIRE(successors
                .filter([&, originalGraph](std::pair<GraphState<std::string,xyLoc>, cost_t> p) {
                    return originalGraph->getVertex(p.first.getPosition()) == xyLoc{0,3};
                }).size() == 1);
            REQUIRE(successors
                .filter([&, originalGraph](std::pair<GraphState<std::string,xyLoc>, cost_t> p) {
                    return originalGraph->getVertex(p.first.getPosition()) == xyLoc{1,2};
                }).size() == 1);
            REQUIRE(successors
                .filter([&, originalGraph](std::pair<GraphState<std::string,xyLoc>, cost_t> p) {
                    return originalGraph->getVertex(p.first.getPosition()) == xyLoc{1,4};
                }).size() == 1);

            REQUIRE(successors
                .filter([&, originalGraph](std::pair<GraphState<std::string,xyLoc>, cost_t> p) {
                    return originalGraph->getVertex(p.first.getPosition()) == xyLoc{0,2};
                }).size() == 1);
            REQUIRE(successors
                .filter([&, originalGraph](std::pair<GraphState<std::string,xyLoc>, cost_t> p) {
                    return originalGraph->getVertex(p.first.getPosition()) == xyLoc{0,4};
                }).size() == 1);
            REQUIRE(successors
                .filter([&, originalGraph](std::pair<GraphState<std::string,xyLoc>, cost_t> p) {
                    return originalGraph->getVertex(p.first.getPosition()) == xyLoc{2,4};
                }).size() == 1);
        }

        WHEN("testing GraphStateGoalChecker") {
            StandardLocationGoalChecker<GraphState<std::string, xyLoc, cost_t>> goalChecker{};

            GraphState<std::string, xyLoc> n1 = supplier.getState(graph->idOfVertex(xyLoc{1,3}));
            GraphState<std::string, xyLoc> n2 = supplier.getState(graph->idOfVertex(xyLoc{2,3}));
            GraphState<std::string, xyLoc> n3 = supplier.getState(graph->idOfVertex(xyLoc{4,4}));
            GraphState<std::string, xyLoc> n4 = supplier.getState(graph->idOfVertex(xyLoc{4,4}));
            info("werty2");
            GraphState<std::string, xyLoc> goal = supplier.getState(graph->idOfVertex(xyLoc{4,4}));
            info("werty1");

            //location and time do not match
            info("n1");
            REQUIRE_FALSE(goalChecker.isGoal(n1, &goal));
            
            //timestamp match
            info("n2");
            REQUIRE_FALSE(goalChecker.isGoal(n2, &goal));
            
            //location match
            info("n3");
            REQUIRE(goalChecker.isGoal(n3, &goal));
            
            //everything match
            info("n4");
            REQUIRE(goalChecker.isGoal(n4, &goal));
        }

    }

}

SCENARIO("test CpdSearch with optimality bound") {
   
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
         */

        // INCLUDE THE CPD

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./square03.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();

        // CREATE THE TIME GRAPH WITH PERTURBATIONS

        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) {return PerturbatedCost{c, false}; })};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 0}), perturbatedGraph.idOfVertex(xyLoc{2, 0}), PerturbatedCost{150, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{2, 4}), perturbatedGraph.idOfVertex(xyLoc{3, 4}), PerturbatedCost{200, true});

        // USE THE FACTORY TO PROVIDE CpdSearch

        CpdSearchFactory factory{};
        auto factory_output = factory.get(cpdManager, perturbatedGraph, 1);
        CountCpdSearchListener<std::string, xyLoc> listener{};
        factory_output->search.setListener(listener);

        REQUIRE(g.haveSameVertices(perturbatedGraph));

        WHEN("start == goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0})
            ));
            REQUIRE(solution->getCost() == 0);
            REQUIRE(listener.getNodeExpanded() == 0);
        }

        WHEN("start is diagonal goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{1,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{1,1})
            ));
            REQUIRE(solution->getCost() == 141);
            REQUIRE(listener.getNodeExpanded() == 1);
        }

        WHEN("start is above goal (no perturbations)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,1};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(std::make_tuple(xyLoc{0,0}), std::make_tuple(xyLoc{0,1})
            ));
            REQUIRE(solution->getCost() == 100);
            REQUIRE(listener.getNodeExpanded() == 1);
        }

        WHEN("start to goal via perturbated edge") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{0,1}),
                    std::make_tuple(xyLoc{0,2}),
                    std::make_tuple(xyLoc{0,3}),
                    std::make_tuple(xyLoc{0,4})
            ));
            REQUIRE(solution->getCost() == 400);
        }

        WHEN("early termination (perturbation near the end)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
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

SCENARIO("test CpdSearch for suboptimality solutions") {
   
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
         */

        // INCLUDE THE CPD

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./square03.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();

        // CREATE THE TIME GRAPH WITH PERTURBATIONS

        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) { return PerturbatedCost{c, false}; })};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 0}), perturbatedGraph.idOfVertex(xyLoc{2, 0}), PerturbatedCost{500, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 1}), perturbatedGraph.idOfVertex(xyLoc{2, 0}), PerturbatedCost{500, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{2, 4}), perturbatedGraph.idOfVertex(xyLoc{3, 4}), PerturbatedCost{200, true});

        // USE THE FACTORY TO PROVIDE TimeCpdSearch

        CpdSearchFactory factory{};
        auto factory_output = factory.get(cpdManager, perturbatedGraph, 5);

        REQUIRE(g.haveSameVertices(perturbatedGraph));

        WHEN("start == goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId);
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
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}), 
                    std::make_tuple(xyLoc{1,0}),
                    std::make_tuple(xyLoc{2,0}),
                    std::make_tuple(xyLoc{3,0}),
                    std::make_tuple(xyLoc{4,0})
            ));
            REQUIRE(solution->getCost() == (3*100 + 1*500));
        }


        WHEN("early termination (perturbation near the end)") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
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

        WHEN("testing bound") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{2,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId);
            GraphState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId); 
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphState<std::string, xyLoc, PerturbatedCost>* x) {
                    return x->getPayload();
                }) == vectorplus<std::tuple<xyLoc>>::make(
                    std::make_tuple(xyLoc{0,0}),
                    std::make_tuple(xyLoc{1,0}),
                    std::make_tuple(xyLoc{2,0})
            ));
            REQUIRE(solution->getCost() == (1*100 + 1*500));
        }

        delete factory_output;
    }
}

SCENARIO("test CPdFocalHeuristic") {

    GIVEN("a gridmap") {

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
         * 0.....
         * 1...@@
         * 2..@@.
         * 3...@@
         * 4.....
         * 
         */

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./square03.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& reorderedGraph = cpdManager.getReorderedGraph();
        // IT'S REALLY IMPORTANT THAT WE QUERY NOT ON "graph" BUT ON "reorderedGraph"!!!!!
        // THIS because the CPD uses those indices, not the ones from "graph"!!!
        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) {return PerturbatedCost{c, false}; })};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{0,3}), perturbatedGraph.idOfVertex(xyLoc{0,4}), PerturbatedCost{200, true});

        CpdFocalHeuristic<GraphState<std::string, xyLoc>, std::string, xyLoc> h{
            cpdManager, 
            perturbatedGraph
        };

        REQUIRE(h.isAdmissible());
        REQUIRE(h.isConsistent());

        WHEN("start=goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,0};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 0);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == 0);
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == 0);
        }

        WHEN("start above goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{0,1};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 100);
            REQUIRE(h.getCachedElementsNumber() == 2);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == (1*100));
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(startLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(startLoc)) == (1*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == (0*100));
        }

        WHEN("start diagonal goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{1,1};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 141);
            REQUIRE(h.getCachedElementsNumber() == 2);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == (1*141));
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(startLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(startLoc)) == (1*141));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == (0*141));
        }

        WHEN("start above goal with perturbation") {
            xyLoc startLoc{0,3};
            xyLoc goalLoc{0,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 100);
            REQUIRE(h.getCachedElementsNumber() == 2);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == 0);
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(startLoc));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(startLoc)) == reorderedGraph.idOfVertex(startLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(startLoc)) == (0*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == (0*100));
        }

        WHEN("start distance from goal of 2") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{2,0};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 200);
            REQUIRE(h.getCachedElementsNumber() == 3);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == (2*100));
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(startLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(startLoc)) == (2*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == (0*100));
        }

        WHEN("start diangonal distance from goal of 2") {
            xyLoc startLoc{0,2};
            xyLoc goalLoc{2,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == 282);
            REQUIRE(h.getCachedElementsNumber() == 3);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == (2*141));
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(startLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(startLoc)) == (2*141));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == (0*141));
        }


        WHEN("start distance from goal with perturbation") {
            xyLoc startLoc{0,4};
            xyLoc goalLoc{0,1};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == (3*100));
            REQUIRE(h.getCachedElementsNumber() == 4);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == (0));
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(startLoc));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(startLoc)) == reorderedGraph.idOfVertex(startLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(startLoc)) == (0*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(xyLoc{0,3})) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(xyLoc{0,3})) == (2*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(xyLoc{0,2})) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(xyLoc{0,2})) == (1*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == (0*100));
        }

        WHEN("start distance from goal with perturbation") {
            xyLoc startLoc{0,1};
            xyLoc goalLoc{0,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == (3*100));
            REQUIRE(h.getCachedElementsNumber() == 4);
            REQUIRE(h.getLastEarliestPerturbationSourceIdCost() == (2*100));
            REQUIRE(h.getLastEarliestNodeBeforePerturbation() == reorderedGraph.idOfVertex(xyLoc{0,3}));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(startLoc)) == reorderedGraph.idOfVertex(xyLoc{0,3}));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(startLoc)) == (2*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(xyLoc{0,2})) == reorderedGraph.idOfVertex(xyLoc{0,3}));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(xyLoc{0,2})) == (1*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(xyLoc{0,3})) == reorderedGraph.idOfVertex(xyLoc{0,3}));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(xyLoc{0,3})) == (0*100));
            REQUIRE(h.getPerturbatedSourceId(reorderedGraph.idOfVertex(goalLoc)) == reorderedGraph.idOfVertex(goalLoc));
            REQUIRE(h.getPerturbatedPathCost(reorderedGraph.idOfVertex(goalLoc)) == (0*100));
        }

        WHEN("start far from goal") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == (100*4 + 141*2));
            REQUIRE(h.getCachedElementsNumber() == 7);
        }

        WHEN("reusing heuristic and paths are not overlapping (except goal") {
            xyLoc startLoc1{0,0};
            xyLoc startLoc2{4,0};
            xyLoc goalLoc{2,0};
            GraphState<std::string, xyLoc> start1{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc1)};
            GraphState<std::string, xyLoc> start2{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc2)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start1, &goal) == (100*2));
            REQUIRE(h.getHeuristic(start2, &goal) == (100*2));

            REQUIRE(h.getCachedElementsNumber() == 5);
        }

        //TODO coniotnue
        WHEN("reusing heuristic and paths are overlapping in the middle") {
            xyLoc startLoc1{0,0};
            xyLoc startLoc2{2,1};
            xyLoc goalLoc{4,0};
            GraphState<std::string, xyLoc> start1{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc1)};
            GraphState<std::string, xyLoc> start2{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc2)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start1, &goal) == (100*4));
            REQUIRE(h.getHeuristic(start2, &goal) == (100*3));
            REQUIRE(h.getCachedElementsNumber() == 6);
        }

        WHEN("reusing heuristic and one path is subset of the other") {
            xyLoc startLoc1{0,0};
            xyLoc startLoc2{2,0};
            xyLoc goalLoc{4,0};
            GraphState<std::string, xyLoc> start1{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc1)};
            GraphState<std::string, xyLoc> start2{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc2)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start1, &goal) == (100*4));
            REQUIRE(h.getHeuristic(start2, &goal) == (100*2));
            REQUIRE(h.getCachedElementsNumber() == 5);
        }

        WHEN("performing cleanup") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,4};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            h.getHeuristic(start, &goal);

            REQUIRE(h.getCachedElementsNumber() == 7);
            h.cleanup();
            REQUIRE(h.getCachedElementsNumber() == 0);
        }

        WHEN("goal unreachable") {
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,2};
            GraphState<std::string, xyLoc> start{0, reorderedGraph, reorderedGraph.idOfVertex(startLoc)};
            GraphState<std::string, xyLoc> goal{0, reorderedGraph, reorderedGraph.idOfVertex(goalLoc)};
            REQUIRE(h.getHeuristic(start, &goal) == cost_t::INFTY);
            REQUIRE(h.getCachedElementsNumber() == 0);
        }
    }

}

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

        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) {return PerturbatedCost{c, false}; })};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 0}), perturbatedGraph.idOfVertex(xyLoc{2, 0}), PerturbatedCost{150, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{2, 4}), perturbatedGraph.idOfVertex(xyLoc{3, 4}), PerturbatedCost{200, true});

        // USE THE FACTORY TO PROVIDE CpdSearch

        CpdFocalSearchFactory factory{};
        auto factory_output = factory.get(cpdManager, perturbatedGraph, cost_t{1}, 1);

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
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(solution->getCost() == 400);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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

        AdjacentGraph<std::string, xyLoc, PerturbatedCost> perturbatedGraph{*cpdManager.getReorderedGraph().mapEdges<PerturbatedCost>([&](cost_t c) {return PerturbatedCost{c, false}; })};

        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{1, 0}), perturbatedGraph.idOfVertex(xyLoc{2, 0}), PerturbatedCost{150, true});
        perturbatedGraph.changeWeightUndirectedEdge(perturbatedGraph.idOfVertex(xyLoc{2, 4}), perturbatedGraph.idOfVertex(xyLoc{3, 4}), PerturbatedCost{200, true});

        // USE THE FACTORY TO PROVIDE CpdSearch

        CpdFocalSearchFactory factory{};
        //focal bound set to 2 ==> WA*
        auto factory_output = factory.get(cpdManager, perturbatedGraph, fractional_number<cost_t>{2}, 1);

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
            xyLoc startLoc{0,0};
            xyLoc goalLoc{4,0};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(solution->getCost() == 400);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, true);
            REQUIRE(
                solution->map<std::tuple<xyLoc>>([&](const GraphFocalState<std::string, xyLoc, PerturbatedCost>* x) {
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

SCENARIO("test CpdFocalSearch in actual failed tests") {
   
    GIVEN("failed test 1") {

        // CREATE GRAPH WHERE WE WANT TO OPERATE

        MovingAIGridMapReader reader{
            '.', 1000, color_t::WHITE,
            'T', 1500, color_t::GREEN,
            'S', 2000, color_t::CYAN,
            'W', 2500, color_t::BLUE,
            '@', cost_t::INFTY, color_t::BLACK
        };
        GridMap gridMap = reader.load(boost::filesystem::path{"16room_003.map"});
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
         */

        // INCLUDE THE CPD

        CpdManager<std::string, xyLoc> cpdManager{boost::filesystem::path{"./16room_003.cpd"}, graph};
        const IImmutableGraph<std::string, xyLoc, cost_t>& g = cpdManager.getReorderedGraph();

        // CREATE THE TIME GRAPH WITH PERTURBATIONS

        boost::filesystem::path perturbateEdgesFilename{"./16room_003_01.dat"};
        FILE* f = fopen(perturbateEdgesFilename.native().c_str(), "rb");
        if (f == nullptr) {
            REQUIRE(false);
        }
        SetPlus<Edge<PerturbatedCost>> perturbatedEdges{};
        AdjacentGraph<std::string, xyLoc, PerturbatedCost>* perturbatedGraph = new AdjacentGraph<std::string, xyLoc, PerturbatedCost>{
            *cpdManager.getReorderedGraph().mapEdges(std::function<PerturbatedCost(const cost_t&)>([&](const cost_t& c) { 
                return PerturbatedCost{c, false};
            }))
        };
        cpp_utils::serializers::loadFromFile(f, perturbatedEdges);

        for (auto perturbatedEdge : perturbatedEdges) {
            perturbatedGraph->changeWeightEdge(
                perturbatedEdge.getSourceId(), 
                perturbatedEdge.getSinkId(), 
                perturbatedEdge.getPayload()
            );
        }
        fclose(f);

        // USE THE FACTORY TO PROVIDE CpdFocalSearch

        CpdFocalSearchFactory factory{};
        auto factory_output = factory.get(cpdManager, *perturbatedGraph, fractional_number<cost_t>{1., 1e-3}, 1);

        REQUIRE(g.haveSameVertices(*perturbatedGraph));

        WHEN("from 267,135 to 267, 13") {
            critical("testing failing 1");
            xyLoc startLoc{267, 135};
            xyLoc goalLoc{267, 136};
            nodeid_t startId = g.idOfVertex(startLoc);
            nodeid_t goalId = g.idOfVertex(goalLoc);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& start = factory_output->stateSupplier.getState(startId, generation_enum_t::FROM_INPUT);
            GraphFocalState<std::string, xyLoc, PerturbatedCost>& goal = factory_output->stateSupplier.getState(goalId, generation_enum_t::FROM_INPUT);
            auto solution = factory_output->search.search(start, goal, false, false);
            REQUIRE(solution->getCost() == 1000);
        }
    }
}
