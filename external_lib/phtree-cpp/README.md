**This is a fork of [Improbable's (currently unmaintained) PH-tree](https://github.com/improbable-eng/phtree-cpp)**.

Multi-dimensional / spatial index with very fast insert/erase/relocate operations and scalability with large datasets. 
This library is C++ / header only.

[![Bazel Linux build](https://github.com/tzaeschke/phtree-cpp/actions/workflows/bazel.yml/badge.svg)](https://github.com/tzaeschke/phtree-cpp/actions/)
[![CMake Linux build](https://github.com/tzaeschke/phtree-cpp/actions/workflows/cmake.yml/badge.svg)](https://github.com/tzaeschke/phtree-cpp/actions/)
[![CMake MSBuild 17.3.1](https://github.com/tzaeschke/phtree-cpp/actions/workflows/cmake-windows.yml/badge.svg)](https://github.com/tzaeschke/phtree-cpp/actions/)
[![Bazel MacOS AppleClang 14](https://github.com/tzaeschke/phtree-cpp/actions/workflows/bazel-macos.yml/badge.svg)](https://github.com/tzaeschke/phtree-cpp/actions/)
[![codecov](https://codecov.io/gh/tzaeschke/phtree-cpp/branch/master/graph/badge.svg?token=V5XVRQG754)](https://codecov.io/gh/tzaeschke/phtree-cpp)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# PH-Tree C++

The [PH-Tree](https://tzaeschke.github.io/phtree-site/) is an ordered index on an n-dimensional space 
(quad-/oct-/2^n-tree) where each dimension is (by default)
indexed by a 64bit integer. The index order follows z-order / Morton order. The default implementation is effectively
a 'map', i.e. *each key is associated with at most one value.* For convenience there is also a multimap implementations
that supports multiple entries with identical keys.
Keys are points or boxes in n-dimensional space.

Two strengths of PH-Trees are fast insert/removal operations and scalability with large datasets. It also provides fast
window queries and _k_-nearest neighbor queries, and it scales well with higher dimensions. The default implementation
is limited to 63 dimensions.

The API ist mostly analogous to STL's `std::map` and `std::multimap`, see function descriptions for details.

Theoretical background is listed [here](#theory).

More information about PH-Trees (including a Java implementation) is available [here](http://www.phtree.org).

----------------------------------

## User Guide

### API Usage

* [Key Types](#key-types)
* [Basic operations](#basic-operations)
* [Queries](#queries)
  * [for_each](#for-each-example)
  * [Iterators](#iterator-examples)
  * [Filters](#filters)
  * [Filters for MultiMaps](#filters-for-multimaps)
  * [Distance Functions](#distance-functions)
* [Converters](#converters)
* [Custom Key Types](#custom-key-types)
* [Restrictions](#restrictions)
* [Troubleshooting / FAQ](#troubleshooting-faq)

### Performance

* [When to use a PH-Tree](#when-to-use-a-ph-tree)
* [Optimising Performance](#optimizing-performance)

### Compiling / Building

* [Build system & dependencies](#build-system-and-dependencies)
* [bazel](#bazel)
* [cmake](#cmake)

## Further Resources

* [Theory](#theory)

----------------------------------

## API Usage

<a id="key-types"></a>

#### Key Types

The **PH-Tree Map** has five predefined tree types:

- `PhTreeD` uses `PhPointD` keys, which are vectors/points of 64 bit `double`.
- `PhTreeF` uses `PhPointF` keys, which are vectors/points of 32 bit `float`.
- `PhTreeBoxD` uses `PhBoxD` keys, which consist of two `PhPointD` that define an axis-aligned rectangle/box.
- `PhTreeBoxF` uses `PhBoxF` keys, which consist of two `PhPointF` that define an axis-aligned rectangle/box.
- `PhTree` uses `PhPoint` keys, which are vectors/points of `std::int64`

The **PH-Tree MultiMap** has three predefined tree types:

- `PhTreeMultiMapD` uses `PhPointD` keys, which are vectors/points of 64 bit `double`.
- `PhTreeMultiMapF` uses `PhPointF` keys, which are vectors/points of 32 bit `float`.
- `PhTreeMultiMapBoxD` uses `PhBoxD` keys, which consist of two `PhPointD` that define an axis-aligned rectangle/box.
- `PhTreeMultiMapBoxF` uses `PhBoxF` keys, which consist of two `PhPointF` that define an axis-aligned rectangle/box.
- `PhTreeMultiMap` uses `PhPoint` keys, which are vectors/points of `std::int64`

Additional key types and tree types can be defined easily analogous to the types above, please refer to the declaration of the 
types for an example. Support for custom key classes (points and boxes) as well as custom coordinate mappings can be
implemented using custom `Converter` classes, see below. The `PhTreeMultiMap` is by default backed
by `std::unordered_set` but this can be changed via a template parameter.

The `PhTree` and `PhTreeMultiMap` types are declared in `phtree.h` and `phtree_multimap.h`.

<a id="basic-operations"></a>

#### Basic Operations

```C++
class MyData { ... };
MyData my_data; 

// Create a 3D point tree with floating point coordinates and a value type of `MyData`.
auto tree = PhTreeD<3, MyData>();

// Create coordinate
PhPointD<3> p{1.1, 1.0, 10.};

// Some operations
tree.relocate(p1, p2); // Move an entry from point 1 to point 2
tree.relocate_if(p1, p2, predicate); // Conditionally move an entry from point 1 to point 2
tree.emplace(p, my_data);
tree.emplace_hint(hint, p, my_data);
tree.try_emplace(p, my_data);
tree.try_emplace(hint, p, my_data);
tree.insert(p, my_data);
tree[p] = my_data;
tree.count(p);
tree.find(p);
tree.lower_bounds(p); // Find a key or the next higher key following Morton order (except positive/negative swapped)
tree.erase(p);
tree.erase(iterator);
tree.size();
tree.empty();
tree.clear();

// Multi-map only
tree.estimate_count(query);
```

<a id="queries"></a>

#### Queries

* For-each over all elements: `tree.for_each(callback);`
  **Note that `for_each` tends to be 10%-20% faster than using an iterator.**
* Iterator over all elements: `auto iterator = tree.begin();`
* For-each with box shaped window queries: `tree.for_each(PhBoxD(min, max), callback);`
* Iterator for box shaped window queries: `auto q = tree.begin_query(PhBoxD(min, max));`
* Iterator for _k_ nearest neighbor queries: `auto q = tree.begin_knn_query(k, center_point, distance_function);`
* Custom query shapes, such as spheres: `tree.for_each(callback, FilterSphere(center, radius, tree.converter()));`

<a id="for-each-example"></a>

##### For-each example

```C++
// Callback for counting entries
struct Counter {
    void operator()(PhPointD<3> key, T& t) {
        ++n_;
    }
    size_t n_ = 0;
};

// Count entries inside an axis aligned box defined by the two points (1,1,1) and (3,3,3)
Counter callback;
tree.for_each({{1, 1, 1}, {3, 3, 3}}, callback);
// callback.n_ is now the number of entries in the box.
```

<a id="iterator-examples"></a>

##### Iterator examples

```C++
// Iterate over all entries
for (auto it : tree) {
    ...
}

// Iterate over all entries inside an axis aligned box defined by the two points (1,1,1) and (3,3,3)    
for (auto it = tree.begin_query({{1, 1, 1}, {3, 3, 3}}); it != tree.end(); ++it) {
    ...
}

// Find 5 nearest neighbors of (1,1,1)    
for (auto it = tree.begin_knn_query(5, {1, 1, 1}, DistanceEuclidean<3>())); it != tree.end(); ++it) {
    ...
}
```

<a id="filters"></a>

##### Filters

All queries allow specifying an additional filter. The filter is called for every key/value pair that would normally be
returned (subject to query constraints) and to every node in the tree that the query decides to traverse (also subject
to query constraints). Returning `true` in the filter does not change query behaviour, returning `false` means that the
current value or child node is not returned or traversed. An example of a geometric filter can be found
in `phtree/common/filter.h` in `FilterAABB` or `FilterSphere` (for examples with box keys see
`FilterBoxAABB` or `FilterBoxSphere`).

```C++
template <dimension_t DIM, typename T>
struct FilterByValueId {
    [[nodiscard]] constexpr bool IsEntryValid(const PhPoint<DIM>& key, const T& value) const {
        // Arbitrary example: Only allow values with even values of id_
        return value.id_ % 2 == 0;
    }
    [[nodiscard]] constexpr bool IsNodeValid(const PhPoint<DIM>& prefix, int bits_to_ignore) const {
        // Allow all nodes
        return true;
    }
};

// Iterate over all entries inside an axis aligned box defined by the two points (1,1,1) and (3,3,3).
// Return only entries that suffice the filter condition.    
for (auto it = tree.begin_query({1, 1, 1}, {3, 3, 3}, FilterByValueId<3, T>())); it != tree.end(); ++it) {
    ...
}
```

Note: The filter example works only for the 'map' version of the PH-Tree, such as `PhTree`, `PhTreeD`, ... . Filters for
the `PhTreeMultiMap` are discussed in the next section.

<a id="filters-for-multimaps"></a>

#### Filters for MultiMaps

The `PhTreeMultiMap` requires a different type of filter. In order to function as a multimap, it uses a collections
("buckets") as entries for each occupied coordinate. The buckets allow it to store several values per coordinate. When
using a filter, the PH-Tree will check `IsEntryValid` for every *bucket* (this is different from version 1.x.x where it
called `IsEntryValid` for every entry in a bucket but never for the bucket itself). Since 2.0.0 there is a new function
required in every multimap filter: `IsBucketEntryValid`. It is called once for every entry in a bucket if the bucket
passed `IsEntryValid`. An example of a geometric filter can be found in `phtree/common/filter.h` in `FilterMultiMapAABB`
.

```C++
template <dimension_t DIM, typename T>
struct FilterMultiMapByValueId {
    template <typename BucketT>
    [[nodiscard]] constexpr bool IsEntryValid(const PhPoint<DIM>& key, const BucketT& bucket) const {
        // Arbitrary example: Only allow keys/buckets with a certain property, e.g. keys that lie within a given sphere.
        return check_some_geometric_propert_of_key(key);
    }
    [[nodiscard]] constexpr bool IsBucketEntryValid(const PhPoint<DIM>& key, const T& value) const {
        // Arbitrary example: Only allow values with even values of id_
        return value.id_ % 2 == 0;
    }
    [[nodiscard]] constexpr bool IsNodeValid(const PhPoint<DIM>& prefix, int bits_to_ignore) const {
        // Allow all nodes
        return true;
    }
};
```

<a id="distance-functions"></a>

##### Distance function

Nearest neighbor queries can also use other distance metrics, such as L1 or Chebyshev distance. Note that the query 
returns a special iterator that provides a function to get the distance of the current entry:

```C++
#include "phtree/phtree.h"

// Find 5 nearest neighbors of (1,1,1) using L1 distance    
for (auto it = tree.begin_knn_query(5, {1, 1, 1}, DistanceL1<3>())); it != tree.end(); ++it) {
    std::cout << "distance = " << it.distance() << std::endl;
    ...
}

```

<a id="converters"></a>

#### Converters

The PH-Tree can internally only process integer keys. In order to use floating point coordinates, the floating point
coordinates must be converted to integer coordinates. The `PhTreeD` and `PhTreeBoxD` use by default the
`PreprocessIEEE` & `PostProcessIEEE` functions. The `IEEE` processor is a loss-less converter (in terms of numeric
precision) that simply takes the 64bits of a double value and treats them as if they were a 64bit integer
(it is slightly more complicated than that, see discussion in the papers referenced above). In other words, it treats
the IEEE 754 representation of the double value as integer, hence the name `IEEE` converter.

The `IEEE` conversion is fast and reversible without loss of precision. However, it has been shown that other converters
can result in indexes that are up to 20% faster. One useful alternative is a `Multiply` converter that convert floating
point to integer by multiplication and casting:

```C++
double my_float = ...;
// Convert to int 
std::int64_t my_int = (std::int64_t) my_float * 1000000.;

// Convert back
double resultung_float = ((double)my_int) / 1000000.;  
```

It is obvious that this approach leads to a loss of numerical precision. Moreover, the loss of precision depends on the
actual range of the double values and the constant. The chosen constant should probably be as large as possible but
small enough such that converted values do not exceed the 64bit limit of `std::int64_t`. Note that the PH-Tree provides
several `ConverterMultiply` implementations for point/box and double/float. For example:

```C++
// Multiply converter that multiplies by 1'000'000 (and divides by 1).
auto tree = PhTreeD<DIM, T, ConverterMultiply<DIM, 1000000, 1>>();
```

You can also write your own converter. For example:

```C++
template <dimension_t DIM>
struct MyConverterMultiply : public ConverterPointBase<DIM, double, scalar_64_t> {
    explicit MyConverterMultiply(double multiplier)
    : multiplier_{multiplier}, divider_{1. / multiplier} {}

    [[nodiscard]] PhPoint<DIM> pre(const PhPointD<DIM>& point) const {
        PhPoint<DIM> out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out[i] = point[i] * multiplier_;
        }
        return out;
    }

    [[nodiscard]] PhPointD<DIM> post(const PhPoint<DIM>& in) const {
        PhPointD<DIM> out;
        for (dimension_t i = 0; i < DIM; ++i) {
            out[i] = ((double)in[i]) * divider_;
        }
        return out;
    }

    [[nodiscard]] auto pre_query(const PhBoxD<DIM>& query_box) const {
        return PhBox{pre(query_box.min()), pre(query_box.max())};
    }

    const double multiplier_;
    const double divider_;
};

template <dimension_t DIM, typename T>
using MyTree = PhTreeD<DIM, T, MyConverterMultiply<DIM>>;

void test() {
    MyConverterMultiply<3> converter{1000000};
    MyTree<3, MyData> tree(converter);
    ...  // use the tree
}
```  

It is also worth trying out constants that are 1 or 2 orders of magnitude smaller or larger than this maximum value.
Experience shows that this may affect query performance by up to 10%. This is due to a more compact structure of the
resulting index tree.

<a id="custom-key-types"></a>

##### Custom key types

With custom converters it is also possible to use your own custom classes as keys (instead of `PhPointD` or `PhBoxF`).
The following example defined custom `MyPoint` and `MyBox` types and a converter that allows using them with a `PhTree`:

```c++
struct MyPoint {
    double x_;
    double y_;
    double z_;
};

using MyBox = std::pair<MyPoint, MyPoint>;

class MyConverterMultiply : public ConverterBase<3, 3, double, scalar_64_t, MyPoint, MyBox> {
    using BASE = ConverterPointBase<3, double, scalar_64_t>;
    using PointInternal = typename BASE::KeyInternal;
    using QueryBoxInternal = typename BASE::QueryBoxInternal;

  public:
    explicit MyConverterMultiply(double multiplier = 1000000)
    : multiplier_{multiplier}, divider_{1. / multiplier} {}

    [[nodiscard]] PointInternal pre(const MyPoint& point) const {
        return {static_cast<long>(point.x_ * multiplier_),
                static_cast<long>(point.y_ * multiplier_),
                static_cast<long>(point.z_ * multiplier_)};
    }

    [[nodiscard]] MyPoint post(const PointInternal& in) const {
        return {in[0] * divider_, in[1] * divider_, in[2] * divider_};
    }

    [[nodiscard]] QueryBoxInternal pre_query(const MyBox& box) const {
        return {pre(box.first), pre(box.second)};
    }

  private:
    const double multiplier_;
    const double divider_;
};

void test() {
    MyConverterMultiply tm;
    PhTree<3, Id, MyConverterMultiply> tree(tm);
    ... // use the tree
}
```

<a id="restrictions"></a>

#### Restrictions

* **C++**: Supports value types of `T` and `T*`, but not `T&`
* **C++**: Return types of `find()`, `emplace()`, ... differ slightly from `std::map`, they have function `first()`
  , `second()` instead of fields of the same name.
* **General**: PH-Trees are **maps**, i.e. each coordinate can hold only *one* entry. In order to hold multiple values
  per coordinate please use the `PhTreeMultiMap` implementations.
* **General**: PH-Trees order entries internally in z-order (Morton order). However, the order is based on the (
  unsigned) bit representation of keys, so negative coordinates are returned *after* positive coordinates.
* **General**: The current implementation support between 2 and 63 dimensions.
* **Differences to std::map**: There are several differences to `std::map`. Most notably for the iterators:
    * `begin()`/`end()` are not comparable with `<` or `>`. Only `it == tree.end()` and `it != tree.end()` is supported.
    * Value of `end()`: The tree has no linear memory layout, so there is no useful definition of a pointer pointing _
      after_ the last entry or any entry. This should be irrelevant for normal usage.

<a id="troubleshooting-faq"></a>

### Troubleshooting / FAQ

**Problem**: The PH-Tree appears to be losing updates/insertions.

**Solution**: Remember that the PH-Tree is a *map*, keys will not be inserted if an identical key already exists. The
easiest solution is to use one of the `PhTreeMultiMap` implementations. Alternatively, this can be solved by turning a
`PhTree` into a multi-map, for example by using something like `std::map` or `std::set` as member type:
`PhTree<3, T, CONVERTER, std::set<MyDataClass>>`. The `set` instances can then be used to handle key conflicts by 
storing multiple entries for the same key. The logic to handle conflicts must currently be implemented manually.

----------------------------------

## Performance

<a id="when-to-use-a-ph-tree"></a>

### When to use a PH-Tree

The PH-Tree is a multi-dimensional index or spatial index. This section gives a rough overview how the PH-Tree compares
to other spatial indexes, such as *k*D-trees, R-trees/BV-hierarchies or quadtrees.

Disclaimer: This overview cannot be comprehensive (there are 100s of spatial indexes out there) and performance depends
heavily on the actual dataset, usage patterns, hardware, ... .

**Generally, the PH-Tree tends to have the following advantages:**

* Fast insertion/removal times. While some indexes, such as *k*-D-trees, trees can be build from scratch very fast, they
  tend to be much slower when removing entries or when indexing large datasets. Also, most indexes require
  rebalancing which may result in unpredictable latency (R-trees) or may result in index degradation if delayed
  (*k*D-trees).

* Competitive query performance. Query performance is generally comparable to other index structures. The PH-Tree is
  fast at looking up coordinates but requires more traversal than other indexes. This means it is especially efficient
  if the query results are 'small', e.g. up to 100 results per query.

* Scalability with large datasets. The PH-Tree's insert/remove/query performance tends to scale well to large datasets
  with millions of entries.

* Scalability with the number of dimensions. The PH-Tree has been shown to deal "well" with high dimensional data (
  1000k+ dimensions). What does "well" mean?
    * It works very well for up to 30 (sometimes 50) dimensions. **Please note that the C++ implementation has not been
      optimized nearly as much as the Java implementation.**
    * For more dimensions (Java was tested with 1000+ dimensions) the PH-Tree still has excellent insertion/deletion
      performance. However, the query performance cannot compete with specialised high-dim indexes such as cover-trees
      or pyramid-trees (these tend to be *very slow* on insertion/deletion though).

* Modification operations (insert/delete) in a PH-Tree are guaranteed to modify only one Node (potentially
  creating/deleting a second one). This guarantee can have advantages for concurrent implementations or when serializing
  the index. Please note that this advantage is somewhat theoretical because this guarantee is not exploited by the
  current implementation (it doesn't support concurrency or serialization).

**PH-Tree disadvantages:**

* A PH-Tree is a *map*, not a *multi-map*. This project also provides `PhTreeMultiMap` implementations that store a
  hash-set at each coordinate. In practice, the overhead of storing sets appears to be usually small enough to not
  matter much.

* PH-Trees are not very efficient in scenarios where queries tend to return large result sets in the order of 1000 or
  more.

<a id="optimizing-performance"></a>

### Optimizing Performance

There are numerous ways to improve performance. The following list gives an overview over the possibilities.

1) **Use `-O3 -mavx, -mbmi2` compiler flags**. Ensure that vectorization and count trailing zeros (CTZ/TZCNT) are 
   enabled. 

2) **Use `for_each` instead of iterators**. This should improve performance of queries by 10%-20%.

3) **Use `relocate()` / `relocate_if()` if possible**. When updating the position of an entry, the naive way is 
   to use `erase()` / `emplace()`. With `relocate` / `relocate_if()`, insertion can avoid a lot of duplicate 
   navigation in the tree if the new coordinate is close to the old coordinate.
   ```c++
   relocate(old_position, new_position);
   relocate_if(old_position, new_position, [](const T& value) { return [true/false]; });
   ```
   The multi-map version relocates all values unless a 'value' is specified to identify the value to be relocated: 
   ```c++
   relocate(old_position, new_position, value);
   ```

4) **Store pointers instead of large data objects**. For example, use `PhTree<3, MyLargeClass*>` instead of
   `PhTree<3, MyLargeClass>` if `MyLargeClass` is large.
    * This prevents the PH-Tree from storing the values inside the tree. This should improve cache-locality and thus
      performance when operating on the tree.
    * Using pointers is also useful if construction/destruction of values is expensive. The reason is that the tree has
      to construct and destruct objects internally. This may be avoidable but is currently still happening.

5) **Use non-box query shapes**. Depending on the use case it may be more suitable to use a custom filter for queries.
   For example:

   `tree.for_each(callback, FilterSphere(center, radius, tree.converter()));`

6) **Use a different data converter**. The default converter of the PH-Tree results in a reasonably fast index. Its
   biggest advantage is that it provides lossless conversion from floating point coordinates to PH-Tree coordinates
   (integers) and back to floating point coordinates.
    * The `ConverterMultiply` is a lossy converter but it tends to improve performance by 10% or more. This is not
      caused by faster operation in the converter itself but by a more compact tree shape. The example shows how to use
      a converter that multiplies coordinates by 100'000, thus preserving roughly 5 fractional digits:

      `PhTreeD<DIM, T, ConverterMultiply<3, 100 * 1000, 1>>()`

7) **Use custom key types**. By default, the PH-Tree accepts only coordinates in the form of its own key types, such
   as `PhPointD`, `PhBoxF` or similar. To avoid conversion from custom types to PH-Tree key types, custom classes can
   often be adapted to be accepted directly by the PH-Tree without conversion. This requires implementing a custom
   converter as described in the section about [Custom Key Types](#custom-key-types).

8) Advanced: **Adapt internal Node representation**. Depending on the dimensionality `DIM`, the PH-Tree uses 
   internally in `Nodes` different container types to hold entries. 
   By default, it uses an array for `DIM<=3`, a vector for `DIM<=8` and an ordered map for `DIM>8`. 
   Adapting these thresholds can have strong effects on performance as well as memory usage. 
   One example: Changing the threshold to use vector for `DIM==3` reduced performance of the `update_d` benchmark
   by 40%-50% but improved performance of `query_d` by 15%-20%. The threshold is currently hardcoded.     
   The effects are not always easy to predict but here are some guidelines:
    * "array" is the fastest solution for insert/update/remove type operations. Query performance is "ok". Memory
      consumption is
      **O(DIM^2)** for every node regardless of number of entries in the node.
    * "vector" is the fastest for queries but has for large nodes **worst case O(DIM^2)** insert/update/remove
      performance.
    * "map" scales well with `DIM` but is for low values of `DIM` generally slower than "array" or "vector".

----------------------------------

## Compiling the PH-Tree

The PH-Tree index itself is a *header only* library, it can be used by simply copying everything in the 
`include/phtree` folder. 
The examples, tests and benchmarks can be build with bazel or cmake.  

<a id="build-system-and-dependencies"></a>

### Build system & dependencies

PH-tree can be built with [Bazel](https://bazel.build) (primary build system) or with 
[cmake](https://cmake.org/) *3.14*. 
All code is written in C++ targeting the C++17 standard. 
The code has been verified to compile on Ubuntu Linux with Clang 14 and GCC 12, 
on Windows with Visual Studio 2022 (cmake only, and except benchmarks, which don't work with VS) 
and on MacOS with AppleClang 14 (bazel only).  

The PH-tree makes use of vectorization and CountTrailingZeros/CTZ/TZCNT, so suggested compilation options for 
clang/gcc are:
```
-O3 -mavx -mbmi2
```

For Windows, when using the Windows SDK, you may need to define `NOMINMAX` either using the preprocessor `/D` or 
with `#define NOMINMAX` before importing any windows files. Alternatively, you can import PH-tree dependencies
__before__ importing any Windows SDK dependencies. 


<a id="bazel"></a>

### Bazel
`WORKSPACE` file:
```
http_archive(
    name = "phtree",
    strip_prefix = "phtree-cpp-v1.6.1",
    url = "https://github.com/tzaeschke/phtree-cpp",
)
```
`BUILD` file:
```
cc_binary(
    ...
    deps = [
        "@phtree//:phtree",
    ],
)
```

Once you have set up your dependencies, you should be able to build the PH-Tree repository by running:
```
bazel build ...
```

Similarly, you can run all unit tests with:
```
bazel test ...
```

Benchmarks:
```
bazel run //benchmark:update_mm_d_benchmark --config=benchmark  -- --benchmark_counters_tabular=true
```


<a id="cmake"></a>

### cmake dependency
The library supports three types of cmake dependency management, `FetchContent`, `find_package()` and `add_subfolder()`.
All three approaches are used in [this example project](https://github.com/tzaeschke/test-phtree-cpp-cmake).
#### FetchContent
With `FetchContent_...()`:
```
include(FetchContent)
FetchContent_Declare(
        phtree
        GIT_REPOSITORY https://github.com/tzaeschke/phtree-cpp.git
        GIT_TAG v1.6.1
)
FetchContent_MakeAvailable(phtree)
```

#### find_package()
You need to build the library with:
```
mkdir out && cd out
cmake .. -DPHTREE_INSTALL=on
sudo cmake --build . --config Release --target install -- -j <number of cores>
```
Note that the option `CMAKE_INSTALL_PREFIX:PATH=...` does _not_ work.
The library can then be included with:
```
find_package(phtree CONFIG REQUIRED)
add_executable(ExampleProject example.cc)
target_link_libraries(ExampleProject phtree::phtree)
```

#### add_subfolder()
For this you can simply copy the PH-Tree source code into your project (you can skip `benchmark` and `test`) and
then include the folder with `add_subdirectory(phtree-cpp)`.


### cmake build
`cmake` uses `ccache` when available.
```
mkdir build
cd build
cmake ..
cmake --build .
```

Run example:
```
cmake .. -DPHTREE_BUILD_EXAMPLES=ON
cmake --build .
./example/Example
```

Run tests:
```
cmake .. -DPHTREE_BUILD_TESTS=ON
cmake --build .
ctest
```
Next to example (`PHTREE_BUILD_EXAMPLES`) there are also tests (`PHTREE_BUILD_TESTS`) and 
benchmarks (`PHTREE_BUILD_BENCHMARKS`). To build all, use `PHTREE_BUILD_ALL`. 
**Note that the benchmarks currently don't work on Windows.** 

## Further Resources

<a id="theory"></a>

### Theory

The PH-Tree is discussed in the following publications and reports:

- T. Zaeschke, C. Zimmerli, M.C. Norrie:
  "The PH-Tree -- A Space-Efficient Storage Structure and Multi-Dimensional Index", (SIGMOD 2014)
- T. Zaeschke: "The PH-Tree Revisited", (2015)
- T. Zaeschke, M.C. Norrie: "Efficient Z-Ordered Traversal of Hypercube Indexes" (BTW 2017).

