/*
 * Copyright 2022 Tilmann Zäschke
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
#include <include/gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <iostream>

using namespace improbable::phtree;

using namespace std;

namespace phtree_test_issues {

#if defined(__clang__) || defined(__GNUC__)

void mem_usage(double& vm_usage, double& resident_set) {
    vm_usage = 0.0;
    resident_set = 0.0;
    ifstream stat_stream("/proc/self/stat", ios_base::in);  // get info from proc directory
    // create some variables to get info
    string pid, comm, state, ppid, pgrp, session, tty_nr;
    string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    string utime, stime, cutime, cstime, priority, nice;
    string O, itrealvalue, starttime;
    unsigned long vsize;
    long rss;
    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >> tpgid >> flags >>
        minflt >> cminflt >> majflt >> cmajflt >> utime >> stime >> cutime >> cstime >> priority >>
        nice >> O >> itrealvalue >> starttime >> vsize >> rss;  // don't care about the rest
    stat_stream.close();
    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;  // for x86-64 is configured to use 2MB pages
    vm_usage = vsize / 1024.0;
    resident_set = rss * page_size_kb;
}

int get_resident_mem_kb() {
    double vm, rss;
    mem_usage(vm, rss);
    return rss;
}

void print_mem() {
    double vm, rss;
    mem_usage(vm, rss);
    cout << "  Virtual Memory: " << vm << " KB" << std::endl
         << "  Resident set size: " << rss << " KB" << endl;
}

#elif defined(_MSC_VER)
int get_resident_mem_kb() {
    return 0;
}

void print_mem() {
    double vm = 0, rss = 0;
    // mem_usage(vm, rss);
    cout << "  Virtual Memory: " << vm << " KB" << std::endl
         << "  Resident set size: " << rss << " KB" << endl;
}
#endif

auto start_timer() {
    return std::chrono::steady_clock::now();
}

template <typename T>
void end_timer(T start, const char* prefix) {
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds1 = end - start;
    std::cout << "elapsed time " << prefix << " = " << elapsed_seconds1.count() << " s"
              << std::endl;
}

// Disabled for cmake CI builds because it always fails
#if !defined(SKIP_TEST_MEMORY_LEAKS)
TEST(PhTreeTestIssues, TestIssue60) {
    // auto tree = PhTreeMultiMapD<2, int>();
    auto tree = PhTreeMultiMapD<2, int, ConverterIEEE<2>, std::set<int>>();
    std::vector<PhPointD<2>> vecPos;
    int dim = 1000;
    int num = 1000;

    auto start1 = start_timer();
    for (int i = 0; i < num; ++i) {
        PhPointD<2> p = {(double)(rand() % dim), (double)(rand() % dim)};
        vecPos.push_back(p);
        tree.emplace(p, i);
    }
    end_timer(start1, "1");

    // "warm up": relocate() will inevitably allocate a little bit of memory (new nodes etc).
    // This warm up allocates this memory before we proceed to leak testing which ensures that the
    // memory does not grow.
    for (int j = 0; j < 100; ++j) {
        for (int i = 0; i < num; ++i) {
            PhPointD<2>& p = vecPos[i];
            PhPointD<2> newp = {(double)(rand() % dim), (double)(rand() % dim)};
            tree.relocate(p, newp, i);
            p = newp;
        }
    }

    // Leak testing
    print_mem();
    auto start2 = start_timer();
    auto mem_start_2 = get_resident_mem_kb();
    for (int j = 0; j < 100; ++j) {
        for (int i = 0; i < num; ++i) {
            PhPointD<2>& p = vecPos[i];
            PhPointD<2> newp = {(double)(rand() % dim), (double)(rand() % dim)};
            tree.relocate(p, newp, i);
            p = newp;
        }
    }
    end_timer(start2, "2");

    auto mem_end_2 = get_resident_mem_kb();
    ASSERT_LT(abs(mem_end_2 - mem_start_2), 1);
    print_mem();
}
#endif

// Disabled for cmake CI builds because it always fails
#if !defined(SKIP_TEST_MEMORY_LEAKS)
TEST(PhTreeTestIssues, TestIssue60_minimal) {
    // auto tree = PhTreeMultiMapD<2, int>();
    auto tree = PhTreeMultiMapD<2, int, ConverterIEEE<2>, std::set<int>>();
    std::vector<PhPointD<2>> vecPos;
    int dim = 1000;
    int num = 1000;

    auto start1 = start_timer();
    for (int i = 0; i < num; ++i) {
        PhPointD<2> p = {(double)(rand() % dim), (double)(rand() % dim)};
        vecPos.push_back(p);
        tree.emplace(p, i);
    }
    end_timer(start1, "1");

    // "warm up": relocate() will inevitably allocate a little bit of memory (new nodes etc).
    // This warm up allocates this memory before we proceed to leak testing which ensures that the
    // memory does not grow.
    for (int j = 0; j < 100; ++j) {
        for (int i = 0; i < num; ++i) {
            PhPointD<2>& p = vecPos[i];
            PhPointD<2> newp = {(double)(rand() % dim), (double)(rand() % dim)};
            tree.relocate(p, newp, i);
            p = newp;
        }
    }

    // Leak testing
    print_mem();
    auto start2 = start_timer();
    auto mem_start_2 = get_resident_mem_kb();
    for (int j = 0; j < 100; ++j) {
        for (int i = 0; i < num; ++i) {
            PhPointD<2>& p = vecPos[i];
            PhPointD<2> newp = {p[0] + 1, p[1] + 1};
            tree.relocate(p, newp, i);
            p = newp;
        }
    }
    end_timer(start2, "2");

    auto mem_end_2 = get_resident_mem_kb();
    ASSERT_LT(abs(mem_end_2 - mem_start_2), 1);
    print_mem();
}
#endif

TEST(PhTreeTestIssues, TestIssue6_3_MAP) {
    auto tree = PhTreeD<2, int>();
    std::vector<PhPointD<2>> vecPos;
    int dim = 10000;

    int num = 100000;
    for (int i = 0; i < num; ++i) {
        PhPointD<2> p = {(double)(rand() % dim), (double)(rand() % dim)};
        vecPos.push_back(p);
        tree.emplace(p, i);
    }

    print_mem();
    for (int i = 0; i < num; ++i) {
        PhPointD<2> p = vecPos[i];
        PhPointD<2> newp = {(double)(rand() % dim), (double)(rand() % dim)};
        tree.relocate(p, newp);
    }
    print_mem();
}

}  // namespace phtree_test_issues
