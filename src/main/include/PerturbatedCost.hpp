#ifndef _CPD_SEARCH_PERTURBATED_COST_HEADER__
#define _CPD_SEARCH_PERTURBATED_COST_HEADER__

#include <pathfinding-utils/types.hpp>
#include <cstdio>

#include <pathfinding-utils/operators.hpp>

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
        PerturbatedCost();
        PerturbatedCost(cost_t cost, bool perturbated);
        virtual ~PerturbatedCost();
        PerturbatedCost(const PerturbatedCost& other);
        PerturbatedCost(PerturbatedCost&& other);
        PerturbatedCost& operator =(const PerturbatedCost& other);
        PerturbatedCost& operator =(PerturbatedCost&& other);
    public:
        friend bool operator ==(const PerturbatedCost& a, const PerturbatedCost& b);
        friend std::ostream& operator <<(std::ostream& ss, const PerturbatedCost& a);
    public:
        static cost_t getCost(const PerturbatedCost& c);
        cost_t getCost() const;
        bool isPerturbated() const;
        bool isUnaffected() const;
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

namespace pathfinding {

    template <>
    struct GetCost<search::PerturbatedCost> {
        cost_t operator() (const pathfinding::search::PerturbatedCost& e) const {
            return e.getCost();
        }
    };
}

#endif