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

        PerturbatedCost::PerturbatedCost() : cost{0}, perturbated{false} {

        }

        PerturbatedCost::PerturbatedCost(cost_t cost, bool perturbated): cost{cost}, perturbated{perturbated} {

        }

        PerturbatedCost::~PerturbatedCost() {

        }

        PerturbatedCost::PerturbatedCost(const PerturbatedCost& other): cost{other.cost}, perturbated{other.perturbated} {

        }

        PerturbatedCost::PerturbatedCost(PerturbatedCost&& other): cost{std::move(other.cost)}, perturbated{std::move(other.perturbated)} {

        }

        PerturbatedCost& PerturbatedCost::operator =(const PerturbatedCost& other) {
            this->cost = other.cost;
            this->perturbated = other.perturbated;
            return *this;
        }

        PerturbatedCost& PerturbatedCost::operator =(PerturbatedCost&& other) {
            this->cost = std::move(other.cost);
            this->perturbated = std::move(other.perturbated);
            return *this;
        }

        bool operator ==(const PerturbatedCost& a, const PerturbatedCost& b) {
            return a.cost == b.cost && a.perturbated == b.perturbated;
        }
        
        std::ostream& operator <<(std::ostream& ss, const PerturbatedCost& a) {
            ss << "{" << a.cost << " " << (a.perturbated ? "perturbated" : "unaffected") << "}";
            return ss;
        }
    
        cost_t PerturbatedCost::getCost(const PerturbatedCost& c) {
            return c.getCost();
        }

        cost_t PerturbatedCost::getCost() const {
            return this->cost;
        }

        bool PerturbatedCost::isPerturbated() const {
            return this->perturbated;
        }

        bool PerturbatedCost::isUnaffected() const {
            return !this->perturbated;
        }


}