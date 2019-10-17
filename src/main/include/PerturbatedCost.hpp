#ifndef _CPD_SEARCH_PERTURBATED_COST_HEADER__
#define _CPD_SEARCH_PERTURBATED_COST_HEADER__

#include <pathfinding-utils/types.hpp>
#include <cstdio>

namespace pathfinding::search {

    class PerturbatedCost;

}

namespace cpp_utils::serializers {

    void saveToFile(FILE* f, const pathfinding::search::PerturbatedCost& p);
    pathfinding::search::PerturbatedCost& loadFromFile(FILE* f, pathfinding::search::PerturbatedCost& result);
    
}

namespace pathfinding::search {

    using namespace pathfinding;

    /**
     * @brief Represents an edge cost perturbated
     * 
     */
    class PerturbatedCost {
        friend void cpp_utils::serializers::saveToFile(FILE* f, const PerturbatedCost& p);
        friend PerturbatedCost& cpp_utils::serializers::loadFromFile(FILE* f, PerturbatedCost& result);
    private:
        /**
         * @brief the cost of the edge
         * 
         */
        cost_t cost;
        /**
         * @brief true if the cost has been perturbated w.r.t the original map
         * 
         */
        bool perturbated;
    public:
        /**
         * @brief Internal do not use it
         * 
         */
        PerturbatedCost() : cost{0}, perturbated{false} {

        }
        PerturbatedCost(cost_t cost, bool perturbated): cost{cost}, perturbated{perturbated} {

        }
        ~PerturbatedCost() {

        }
        PerturbatedCost(const PerturbatedCost& other): cost{other.cost}, perturbated{other.perturbated} {

        }
        PerturbatedCost(PerturbatedCost&& other): cost{other.cost}, perturbated{other.perturbated} {

        }
        PerturbatedCost& operator =(const PerturbatedCost& other) {
            this->cost = other.cost;
            this->perturbated = other.perturbated;
            return *this;
        }
        PerturbatedCost& operator =(PerturbatedCost&& other) {
            this->cost = other.cost;
            this->perturbated = other.perturbated;
            return *this;
        }
    public:
        friend bool operator ==(const PerturbatedCost& a, const PerturbatedCost& b) {
            return a.cost == b.cost && a.perturbated == b.perturbated;
        }
        friend std::ostream& operator <<(std::ostream& ss, const PerturbatedCost& a) {
            ss << "{" << a.cost << " " << (a.perturbated ? "perturbated" : "unaffected") << "}";
            return ss;
        }
    public:
        static cost_t getCost(const PerturbatedCost& c) {
            return c.getCost();
        }
        cost_t getCost() const {
            return this->cost;
        }
        bool isPerturbated() const {
            return this->perturbated;
        }
        bool isUnaffected() const {
            return !this->perturbated;
        }
    };

}

namespace std {

    template <>
    struct hash<pathfinding::search::PerturbatedCost> {
        size_t operator() (const pathfinding::search::PerturbatedCost& e) const {
            size_t seed = 0;
            boost::hash_combine(seed, e.getCost());
            boost::hash_combine(seed, e.isPerturbated());
            return seed;
        }
    };
}

namespace boost {

    template <>
    struct hash<pathfinding::search::PerturbatedCost> {
        size_t operator() (const pathfinding::search::PerturbatedCost& e) const {
            return ::std::hash<pathfinding::search::PerturbatedCost>{}(e);
        }
    };

}

#endif