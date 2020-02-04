#include "cpd_search_generated_e.hpp"

namespace pathfinding {

    std::vector<const cpd_search_generated_e*> cpd_search_generated_e::VALUES{};

    const cpd_search_generated_e cpd_search_generated_e::INPUT = cpd_search_generated_e{"INPUT"};
    const cpd_search_generated_e cpd_search_generated_e::CPDPATH = cpd_search_generated_e{"CPDPATH"};
    const cpd_search_generated_e cpd_search_generated_e::GRAPHSUCCESSOR = cpd_search_generated_e{"GRAPHSUCCESSOR"};
    const cpd_search_generated_e cpd_search_generated_e::FROM_SOLUTION = cpd_search_generated_e{"FROM_SOLUTION"};

    const cpd_search_generated_e& cpd_search_generated_e::getFirst() {
        return *VALUES[0];
    }

    const std::vector<const cpd_search_generated_e*>& cpd_search_generated_e::getValues() {
        return cpd_search_generated_e::VALUES;
    }

}