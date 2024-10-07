/*
 * Copyright 2020 Improbable Worlds Limited
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

#include "phtree/phtree.h"
#include "phtree/phtree_multimap.h"
#include <chrono>
#include <iostream>
#include <set>

using namespace improbable::phtree;

int relocate_example() {
    //auto tree = PhTreeMultiMapD<2, int, ConverterIEEE<2>, std::unordered_set<int>>();
    auto tree = PhTreeMultiMapD<2, int, ConverterMultiply<2, 1, 200>, std::unordered_set<int>>();
    std::vector<PhPointD<2>> vecPos;
    int dim = 1000;

    int num = 30000;
    for (int i = 0; i < num; ++i) {
        PhPointD<2> p = {(double)(rand() % dim), (double)(rand() % dim)};
        vecPos.push_back(p);
        tree.emplace(p, i);
    }

    long T = 0;
    int nT = 0;
    while (true) {
        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < num; ++i) {
            PhPointD<2>& p = vecPos[i];
            PhPointD<2> newp = {p[0] + 1, p[1] + 1};
            tree.relocate(p, newp, i, false);
            p = newp;
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto s = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
        ++nT;
        T += (long)s.count() / 1000;
        std::cout << s.count() << "    " << (T / nT)
                  << "     msec/num= " << (s.count() / (double)num) << std::endl;
    }

    return 0;
}

int main() {
    std::cout << "PH-Tree example with 3D `double` coordinates." << std::endl;
    PhPointD<3> p1({1, 1, 1});
    PhPointD<3> p2({2, 2, 2});
    PhPointD<3> p3({3, 3, 3});
    PhPointD<3> p4({4, 4, 4});

    PhTreeD<3, int> tree;
    tree.emplace(p1, 1);
    tree.emplace(p2, 2);
    tree.emplace(p3, 3);
    tree.emplace(p4, 4);

    std::cout << "All values:" << std::endl;
    for (auto it : tree) {
        std::cout << "    id=" << it << std::endl;
    }
    std::cout << std::endl;

    std::cout << "All points in range:" << p2 << "/" << p4 << std::endl;
    for (auto it = tree.begin_query({p2, p4}); it != tree.end(); ++it) {
        std::cout << "    " << it.second() << " -> " << it.first() << std::endl;
    }
    std::cout << std::endl;

    std::cout << "PH-Tree is a MAP which means that, like std::map, every position " << std::endl;
    std::cout << " (=key) can have only ONE value." << std::endl;
    std::cout << "Storing multiple values for a single coordinate requires storing " << std::endl;
    std::cout << "lists or sets, for example using PhTree<3, std::vector<int>>." << std::endl;

    PhPointD<3> p4b({4, 4, 4});
    tree.emplace(p4b, 5);
    // Still showing '4' after emplace()
    std::cout << "ID at " << p4b << ": " << tree.find(p4b).second() << std::endl;

    std::cout << "Done." << std::endl;

    //relocate_example();

    return 0;
}
