#ifndef _CPDSEARCH_GENERATIONENUM_HEADER__
#define _CPDSEARCH_GENERATIONENUM_HEADER__

namespace pathfinding::search {

    /**
     * @brief Specifies who has generated the state
     * 
     */
    enum class generation_enum_t {
        /**
         * @brief A* generated this
         * 
         */
        FROM_SEARCH,
        /**
         * @brief the processof generating the actual path from A* search nodes generated this state
         * 
         */
        FROM_SOLUTION,
        /**
         * @brief early termination has generated this
         * 
         */
        FROM_EARLY_TERMINATION,
        /**
         * @brief either the start or the goal state
         * 
         */
        FROM_INPUT
    };

}

#endif