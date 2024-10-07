/*
* Copyright 2023 Tilmann Zäschke
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
 */

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <ostream>

#include <map>

#include "include/phtree/phtree_multimap.h"

// clang++ -g -std=c++17 -fsanitize=address,fuzzer fuzzer/phtree_mm_relocate_fuzzer.cc -I. -I./include


using namespace improbable::phtree;


constexpr bool PRINT = !true;

void print() {}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* Data, size_t Size) {
    assert(Data);

    const dimension_t DIM = 1;

    if (PRINT) {
        std::cout << "TEST(PhTreeMMTest, FuzzTest1) {" << std::endl;
        std::cout << "    const dimension_t DIM = 1;" << std::endl;
        std::cout << "    using Key = PhPoint<DIM>;" << std::endl;
        std::cout << "    using Value = std::uint8_t;" << std::endl;
        std::cout << "    PhTreeMultiMap<DIM, Value, ConverterNoOp<DIM, std::int64_t>> tree{};" << std::endl;
    }

    using Instruction = std::uint8_t;
    using Key = PhPoint<1>;
    using Value = std::uint8_t;

    PhTreeMultiMap<DIM, Value, ConverterNoOp<DIM, std::int64_t>> tree;
    std::multimap<Key, Value> map;

    size_t pos = 0;

    while (pos + 4 < Size) {
        Instruction inst = Data[pos++] % 2;
        Key key{Data[pos++]};
        Key key2{Data[pos++]};
        Value value = Data[pos++];
        switch (inst) {
        case 0: {
            if (PRINT)
                std::cout << "    tree.emplace({" << key[0] << "}, " << (int)value << ");"
                          << std::endl;
            tree.emplace({key[0]}, value);
//            map.emplace(key, value);
            break;
        }
        case 1: {
            if (PRINT)
                std::cout << "    tree.relocate({" << key[0] << "}, {"  << key2[0] << "}, "  << (int)value << ");" << std::endl;
//            tree.erase(key);
//            map.erase(key);
            tree.relocate({key[0]}, {key2[0]}, value);
            break;
        }
//        case 2: {
//            if (PRINT)
//                std::cout << "    auto it = tree.find(" << (int)key << ");" << std::endl;
//            auto it = tree.find(key);
//            if (it != tree.end()) {
//                if (PRINT)
//                    std::cout << "    tree.erase(it);" << std::endl;
//                tree.erase(it);
//            }
//            auto it2 = map.find(key);
//            if (it2 != map.end()) {
//                map.erase(it2);
//            }
//            break;
//        }
//        case 3: {
//            if (PRINT)
//                std::cout << "    auto it = tree.lower_bound(" << (int)key << ");" << std::endl;
//            auto it = tree.lower_bound(key);
//            if (PRINT)
//                std::cout << "    tree.emplace_hint(it, " << (int)key << ", " << (int)value << ");"
//                          << std::endl;
//            tree.emplace_hint(it, key, value);
//            auto it2 = map.lower_bound(key);
//            map.emplace_hint(it2, key, value);
//            break;
//        }
        default:
            std::cout << "Unexpected instruction: " << inst << std::endl;
        }
    }

    //tree._check();

//    for (auto& entry : map) {
//        const Key& vRef = entry.first;
//        Key vMap = tree.find(vRef)->first;
//        assert(vMap == vRef);
//    }
//    for (auto& entry : tree) {
//        Key v = entry.first;
//        const Key& vRef = map.find(v)->first;
//        Key vMap = tree.find(v)->first;
//        assert(vMap == vRef);
//    }
//    assert(tree.size() == map.size());

    return 0;
}
