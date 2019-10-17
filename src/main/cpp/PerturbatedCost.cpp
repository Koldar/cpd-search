#include "PerturbatedCost.hpp"


namespace cpp_utils::serializers {

    void saveToFile(FILE* file, const pathfinding::search::PerturbatedCost& p) {
        if(std::fwrite(&p.cost, sizeof(p.cost), 1, file) != 1) {
            throw cpp_utils::exceptions::FileOpeningException{recoverFilename(file)};
        }
        if(std::fwrite(&p.perturbated, sizeof(p.perturbated), 1, file) != 1) {
            throw cpp_utils::exceptions::FileOpeningException{recoverFilename(file)};
        }
    }
    
    pathfinding::search::PerturbatedCost& loadFromFile(FILE* f, pathfinding::search::PerturbatedCost& p) {
        if(std::fread(&p.cost, sizeof(p.cost), 1, f) != 1) {
            throw cpp_utils::exceptions::FileOpeningException{recoverFilename(f)};
        }
        if(std::fread(&p.perturbated, sizeof(p.perturbated), 1, f) != 1) {
            throw cpp_utils::exceptions::FileOpeningException{recoverFilename(f)};
        }

        return p;
    }

}

namespace pathfinding::search {




}