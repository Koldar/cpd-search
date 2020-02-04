#ifndef _CPDSEARCH_CPDSEARCHGENERATED_HEADER__
#define _CPDSEARCH_CPDSEARCHGENERATED_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

namespace pathfinding {

    using namespace cpp_utils;

    class cpd_search_generated_e: public AbstractEnum<cpd_search_generated_e> {
    private:
        static std::vector<const cpd_search_generated_e*> VALUES;
    private:
        cpd_search_generated_e(const std::string& name): AbstractEnum{name, VALUES} {

        }
    public:
        static const cpd_search_generated_e& getFirst();
        static const std::vector<const cpd_search_generated_e*>& getValues();
    public:
        /**
         * @brief the user has explituicly generated it (start and goal)
         * 
         */
        static const cpd_search_generated_e INPUT;
        /**
         * @brief the state is generated by following the cpd path
         * 
         */
        static const cpd_search_generated_e CPDPATH;
        /**
         * @brief the state is generated by following the underlying graph
         * 
         */
        static const cpd_search_generated_e GRAPHSUCCESSOR;
        /**
         * @brief the state is generated while building the algorithm solution
         * 
         */
        static const cpd_search_generated_e FROM_SOLUTION;
    };


}

#endif