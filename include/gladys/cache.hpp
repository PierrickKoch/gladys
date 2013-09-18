/*
 * cache.hpp
 *
 * Graph Library for Autonomous and Dynamic Systems
 *
 * author:  Arnaud Degroote <arnaud.degroote@laas.fr>
 * created: 2013-08-18
 * license: BSD
 */
#ifndef CACHE_HPP
#define CACHE_HPP

#include <list> 
#include <functional>

namespace gladys {
 
    /*
     * A Simple LRU cache (Last Recently Used), parametrized by K (Key), V
     * (Value), and MAP (the container used for storing the association Key ->
     * Value). Basically, it can be std::map or std::unordered_map
     *
     * The cache is expected to emulate a function from K to V, ie V fn(const K&)
     * It is probably an interesting exercice (with variadic template) to
     * extend it to arbitrary signature, and use tuple of argument as key
     * cache.
     *
     * The size of the cache is controlled at construction.
     */
    template <typename K,  typename V, 
               template<typename...> class MAP 
             > 
    class lru_cache
    { 
        public: 
            typedef K key_type; 
            typedef V value_type; 
            typedef std::function<value_type (const key_type&)> fun_type; 

        private:
            // Key access history, most recent at back 
            typedef std::list<key_type> key_tracker_type; 

            // Key to value and key history iterator 
            typedef MAP< 
                key_type, 
                std::pair< 
                    value_type, 
                    typename key_tracker_type::iterator 
                > 
            > key_to_value_type; 

            fun_type fn;
            const size_t capacity; 

            key_tracker_type key_tracker; 
            key_to_value_type key_to_value; 

            // Record a fresh key-value pair in the cache 
            void insert(const key_type& k, const value_type& v) 
            { 
                if (key_to_value.size() == capacity) 
                    evict(); 

                auto it = key_tracker.insert(key_tracker.end(), k); 
                key_to_value.insert(std::make_pair(k, std::make_pair(v,it)));
            } 

            // Purge the least-recently-used element in the cache 
            void evict() 
            { 
                assert(!key_tracker.empty()); 

                const auto it = key_to_value.find(key_tracker.front()); 
                assert(it!=key_to_value.end()); 

                key_to_value.erase(it); 
                key_tracker.pop_front(); 
            } 

        public:
            lru_cache(fun_type fun, size_t c) : 
                fn(fun), capacity(c)
            {
                assert(capacity != 0);
            }

            value_type operator() (const key_type& k) 
            { 

                const auto it = key_to_value.find(k);
                if (it == key_to_value.end()) 
                {
                    value_type v = fn(k);
                    insert(k, v);
                    return v;
                } else {
                    // Update access history
                    key_tracker.splice(key_tracker.end(),
                                       key_tracker,
                                       it->second.second);
                    return it->second.first;
                }
            } 

            void invalidate() 
            {
                key_tracker.clear();
                key_to_value.clear();
            }
    }; 
 

} // gladys

#endif // CACHE_HPP

