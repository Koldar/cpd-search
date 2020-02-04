#ifndef _CPDSEARCH_DISCOUNTEDCPDSTATE_HEADER__
#define _CPDSEARCH_DISCOUNTEDCPDSTATE_HEADER__

#include <pathfinding-utils/GraphState.hpp>

#include "CpdState.hpp"

namespace pathfinding::search {

    /**
     * @brief a CpdState which heuristic can be discounted by a certain factor
     * 
     * This is used in CpdJumpTrailSearch. states which have been generated via a "jump" with the cpd path
     * are granted a discount. The discount is a value between 0 and 1. It's 1 when the state is a direct adjacent of the starting node whiel it's near 0 when the state is near the earliest perturbation.
     * 
     * ```
     * NODE -> N1 -> N2 -> N3 -> N4 -> N5 -PERTURBATION-> N6
     * ```
     * 
     * NODE is the starting vertex. N1 till N5 belong to the cpd path. N5 has a discount of (e.g.) 0.1, N4 0.2, N3 0.4, N2 0.6 and N1 1.0.
     * 
     * The discount is a multiplier of the heuristic value. To ensure admissability, the coefficient isat most 1. 
     * 
     * Thanks to the CPD, along the other properties, each state know what is the id where the next perturbation is alongside the cost of the path to optimally reach it
     * 
     * @tparam G type of the payload of the whole graph
     * @tparam V type of the each vertex in the graph
     * @tparam E type of each edge in the graph
     */
    template <typename G, typename V, typename E>
    class DiscountedCpdState: public CpdState<G, V, E> {
    public:
        using This = DiscountedCpdState<G, V, E>;
        using Super = CpdState<G, V, E>;
    protected:
        /**
         * @brief the discount factor used to discount the heuristic.
         * 
         * The value belongs in [0, 1]
         */
        double discount;
        
    public:
        DiscountedCpdState(stateid_t id, const IImmutableGraph<G, V, E>& g, nodeid_t location): Super{id, g, location}, discount{1} {

        }
        virtual ~DiscountedCpdState() {

        }
        DiscountedCpdState(const This& other) = delete;
        DiscountedCpdState(This&& other) : Super{::std::move(other)}, 
            discount{other.discount} {

        }

        This& operator =(const This& other) = delete;
        This& operator =(This&& other) {
            Super::operator =(::std::move(other));
            this->discount = std::move(other.discount);

            return *this;
        }
    public:
        This* getParent() {
            return static_cast<This*>(this->parent);
        }
        const This* getParent() const {
            return static_cast<const This*>(this->parent);
        }
        void setParent(This* parent) {
            this->parent = static_cast<This*>(parent);
        }
    public:
        void setDiscount(double c) {
            this->discount = c;
        }
        double getDiscount() const {
            return this->discount;
        }
    public:
        friend bool operator <(const This& a, const This& b) {
            //used in open list
            if (a.getF() < b.getF()) {
                return true;
            } else if (a.getF() == b.getF()) {
                //tie breaking. Choose the state with maximum g
                //return a.getG() > b.getG();
                //tie breaking. Choose the state with smaller cpd perturbated cost
                return a.perturbatedPathCost < b.perturbatedPathCost;
            } else {
                return false;
            }
        }
        friend bool operator ==(const This& a, const This& b) {
            return a.id == b.id && a.position == b.position;
        }
        friend bool operator !=(const This& a, const This& b) {
            return !(a == b);
        }
        friend std::ostream& operator <<(std::ostream& out, const This& state) {
            out 
                << "{ id: " 
                << state.getId() 
                << " node: " 
                << state.graph.getVertex(state.getPosition())
                << " f: "
                << state.getF()
                << " g: "
                << state.getG()
                << " discount: "
                << state.getDiscount()
                << " }";
            return out;
        }
    };

}

#endif