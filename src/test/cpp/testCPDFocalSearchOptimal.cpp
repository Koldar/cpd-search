#include "catch.hpp"
#include "CpdFocalSearchOptimal.hpp"
#include <pathfinding-utils/MovingAIGridMapReader.hpp>
#include <pathfinding-utils/GridMapGraphConverter.hpp>

using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::maps;
using namespace pathfinding::search;


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

        CpdFocalSearchOptimalFactory factory{};
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