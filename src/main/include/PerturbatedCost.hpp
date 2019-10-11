#ifndef _CPD_SEARCH_PERTURBATED_COST_HEADER__
#define _CPD_SEARCH_PERTURBATED_COST_HEADER__

namespace pathfinding::search {

    /**
     * @brief Represents an edge cost perturbated
     * 
     */
    class PerturbatedCost {
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

#endif