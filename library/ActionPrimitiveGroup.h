#pragma once

// STL
#include <tuple>

namespace ManipulationInterface {

template<class...ACTION_PRIMITIVES_TYPES>
class ActionPrimitiveGroup {
    public:
        ActionPrimitiveGroup(ACTION_PRIMITIVES_TYPES&&...action_primitives) 
            : m_action_primitives(std::forward<ACTION_PRIMITIVES_TYPES>(action_primitives)...)
        {}

        template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
        void callActionByType(ARGS_T&&...args) {
            _callByType<0, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
        }

        template <class ACTION_PRIMITIVE_T, typename...ARGS_T>
        void callActionByType(ARGS_T&&...args) const {
            _callActionPrimitiveConst<0, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
        }

        template <uint32_t I, typename...ARGS_T>
        void callActionByIndex(ARGS_T&&...args) {

        }

    private:

        template <uint32_t I, class ACTION_PRIMITIVE_T, typename...ARGS_T>
        void _callByType(ARGS_T&&...args) {
            if constexpr (I < (sizeof...(ACTION_PRIMITIVES_TYPES))) {
                if constexpr (std::is_same<std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>, ACTION_PRIMITIVE_T>::value) {
                    // Calls operator () when the type is found
                    std::get<I>(m_action_primitives)(*m_move_group, std::forward<ARGS_T>(args)...);
                } else {
                    // Not found, continue searching
                    _callByType<I + 1, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
                }
            } else {
                //static_assert(false, "Action primitive type is not found");
            }
        }

        //template <uint32_t I, class ACTION_PRIMITIVE_T, typename...ARGS_T>
        //void _callActionPrimitiveConst(ARGS_T&&...args) const {
        //    if constexpr (I < (sizeof...(ACTION_PRIMITIVES_TYPES))) {
        //        if constexpr (std::is_same<std::tuple_element_t<I, std::tuple<ACTION_PRIMITIVES_TYPES...>>, ACTION_PRIMITIVE_T>::value) {
        //            // Calls operator () when the type is found
        //            std::get<I>(m_action_primitives)(*m_move_group, std::forward<ARGS_T>(args)...);
        //        } else {
        //            // Not found, continue searching
        //            _callActionPrimitiveConst<I + 1, ACTION_PRIMITIVE_T, ARGS_T...>(args...);
        //        }
        //    } else {
        //        //static_assert(false, "Action primitive type is not found");
        //    }
        //}

    private:
        std::tuple<ACTION_PRIMITIVES_TYPES...> m_action_primitives;
};
}