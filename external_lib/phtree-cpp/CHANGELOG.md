# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [Unreleased]
### Fixed
- Fixed links in CHANGELOG and README. [#152](https://github.com/tzaeschke/phtree-cpp/issues/152)
- Compiler errors with AppleClang 14. [#149](https://github.com/tzaeschke/phtree-cpp/issues/149)

## [1.6.1] - 2023-04-13
### Changed
- Renamed `CALLBACK` to `CALLBACK_FN` to avoid conflicts with Windows SDK.
  [#142](https://github.com/tzaeschke/phtree-cpp/issues/142)
- Dead code cleanup.[#144](https://github.com/tzaeschke/phtree-cpp/pull/144

### Removed
- Removed copy constructor/assignment from bplus-maps. Added some fringe case tests.
  [#145](https://github.com/tzaeschke/phtree-cpp/pull/145

## [1.6.0] - 2023-03-20
### Added
- Added benchmark for bit operations in `common`. [#128](https://github.com/tzaeschke/phtree-cpp/pull/128)
- Added `lower_bounds(key)` to API. [#126](https://github.com/tzaeschke/phtree-cpp/issues/126)
- Added `bpt_fixed_vector`, a fixed size flat vector for future use. It can be dropped in for an std::vector.
  [#124](https://github.com/tzaeschke/phtree-cpp/pull/124)
- Added Chebyshev distance metric `DistanceChebyshev`. [#129](https://github.com/tzaeschke/phtree-cpp/pull/139)

### Changed
- Changed `bpt_vector` to use `std::destroy` i.o. default dstr. [#132](https://github.com/tzaeschke/phtree-cpp/pull/132)
- Moved B+trees into own namespace. [#131](https://github.com/tzaeschke/phtree-cpp/pull/131)
- Moved some stuff in `common` into `nemaspace detail`. [#129](https://github.com/tzaeschke/phtree-cpp/issues/129)
- Improved kNN search implementation. This also deprecates the post-increment iterator.
  [#118](https://github.com/tzaeschke/phtree-cpp/issues/118)

### Fixed
- Replaced deprecated `<assert.h>` imports with `<cassert>`. [#134](https://github.com/tzaeschke/phtree-cpp/pull/134)
- Fixes undefined behavior in debug method. [#135](https://github.com/tzaeschke/phtree-cpp/pull/135)
- API: Fixes `relocate()` return type to be `size_t`. [#136](https://github.com/tzaeschke/phtree-cpp/pull/136)
- Fix constness of B+tree `check()`. [#137](https://github.com/tzaeschke/phtree-cpp/pull/137)
- Minor fixes for next release. [#140](https://github.com/tzaeschke/phtree-cpp/pull/140)

## [1.5.0] - 2023-02-09
### Added
- Added B+tree multimap for internal (future) use. [#93](https://github.com/tzaeschke/phtree-cpp/issues/93)
- Added some fuzz tests. Not that these require manual compilation, see [fuzzer/README.md](fuzzer/README.md).
  [#114](https://github.com/tzaeschke/phtree-cpp/pull/114)
- Added float-32 variants to multimap: PhTreeMultiMapF, PhTreeMultiMapBoxF. 
  [#117](https://github.com/tzaeschke/phtree-cpp/pull/117)

### Changed
- Clean up array_map. [#107](https://github.com/tzaeschke/phtree-cpp/issues/107),
- Fixed compatibility with bazel 6.0.0. [#109](https://github.com/tzaeschke/phtree-cpp/issues/109),
- Added missing compiler flag for TZCNT/CTZ (count trailing zeros). This should be much faster on haswell or later CPUs.
  [#103](https://github.com/tzaeschke/phtree-cpp/issues/103),
- Rewrote relocate(). This should be much cleaner now and slightly faster. 
  [#98](https://github.com/tzaeschke/phtree-cpp/pull/98), 
  [#99](https://github.com/tzaeschke/phtree-cpp/pull/99),
  [#101](https://github.com/tzaeschke/phtree-cpp/pull/101),
  [#104](https://github.com/tzaeschke/phtree-cpp/pull/104),
  [#115](https://github.com/tzaeschke/phtree-cpp/issues/115)
- Cleaned up HandleCollision() and key comparison functions. [#97](https://github.com/tzaeschke/phtree-cpp/pull/97)
- Improved performance by eliminating memory indirection for DIM > 3. 
  This was enabled by referencing "Node" directly in "Entry" which was enabled by
  implanting an indirection in array_map. [#96](https://github.com/tzaeschke/phtree-cpp/pull/96)
- Improved performance of window queries by executing them partially as point queries.
  This works best for point datasets, and somewhat for box datasets with "include" queries.
  There is no benefit for "intersection" queries. [#88](https://github.com/tzaeschke/phtree-cpp/issues/88)
- Improved benchmarks for insert and query to use a more compact format. 
  [#91](https://github.com/tzaeschke/phtree-cpp/pull/91)
- Improved performance of window queries by optimizing calculation of min/max masks.
  Improved performance of queries and updates by changing bit-width of min/max masks and
  hc_pos_t. [#95](https://github.com/tzaeschke/phtree-cpp/pull/95)

### Removed
- bazel version requirement file `.bazelversion`. [#89](https://github.com/tzaeschke/phtree-cpp/issues/89)

### Fixed
- Fixed copy cstr/assignment of B+trees, see also #102. [#119](https://github.com/tzaeschke/phtree-cpp/pull/119) 
- Fixed numerous warnings when compiling with MSVC. [#120](https://github.com/tzaeschke/phtree-cpp/issues/120)

## [1.4.0] - 2022-09-09
### Added
- Added build features: [#53](https://github.com/tzaeschke/phtree-cpp/issues/53) 
  - linting for C++ and bazel files.
  - Added CI status badges.
  - Added test coverage
- Added support for cmake `FetchContent`.
  See README for details. [#75](https://github.com/tzaeschke/phtree-cpp/issues/75)
- Added support for cmake `find_packet()` and direct import via `add_sub_directory()`.
  See README for details. [#83](https://github.com/tzaeschke/phtree-cpp/issues/83)

### Changed
- Cleaned up build scripts. [#53](https://github.com/tzaeschke/phtree-cpp/issues/53)
- Fixed code coverage + migrate to linux. [#80](https://github.com/tzaeschke/phtree-cpp/issues/80)
- ***BREAKING CHANGE*** The project has been restructured to have a more "standard" directory structure.
  This affects how **bazel** dependencies work (use `deps = ["@phtree//:phtree",]`) and enables **cmake FetchContent_**.
  See README for details. [#75](https://github.com/tzaeschke/phtree-cpp/issues/75)

### Removed
- Nothing.

### Fixed
- Nothing.

## [1.3.0] - 2022-08-28
### Added
- Added flag to relocate() allow short cutting in case of identical keys.
  [#68](https://github.com/tzaeschke/phtree-cpp/issues/68)
- Added tested support for move-only and copy-only value objects.
  [#56](https://github.com/tzaeschke/phtree-cpp/issues/56)
- Added custom bucket implementation (similar to std::unordered_set). This improves update performance by 5%-20%.
  [#44](https://github.com/tzaeschke/phtree-cpp/issues/44)
- Added `PhTree.relocate(old_key, new_key)` and `PhTree.relocate_if(old_key, new_key, predicate)`. 
  This is **a lot faster** than using other methods. 
  [#43](https://github.com/tzaeschke/phtree-cpp/issues/43)
- Added try_emplace(key, value) and try_emplace(iter_hint, key, value)
  [#40](https://github.com/tzaeschke/phtree-cpp/issues/40)
- Added FilterBoxAABB and FilterSphereAABB as examples for filtering a PH-Tree with box keys
  [#33](https://github.com/tzaeschke/phtree-cpp/issues/33)
### Changed
- Moved tests and benchmarks into separate folders. [#67](https://github.com/tzaeschke/phtree-cpp/pull/67)
- Cleaned up unit tests. [#54](https://github.com/tzaeschke/phtree-cpp/pull/54)
- Simplified internals of `erase()`. [#47](https://github.com/tzaeschke/phtree-cpp/pull/47)
- Removed internal use of `std::optional()` to slightly reduce memory overhead
  [#38](https://github.com/tzaeschke/phtree-cpp/issues/38)
- Removed restrictions on bazel version [#35](https://github.com/tzaeschke/phtree-cpp/issues/35)
- **API BREAKING CHANGE**: API of filters have been changed to be more correct, explicit and flexible.
  [#21](https://github.com/tzaeschke/phtree-cpp/issues/21)
  - Correctness: Converters and distance functions are not copied unnecessarily anymore.
  - Explicit: 
    Filters *must* have a mandatory parameter for a converter reference. This ensures that the correct
            converter is used, probably `tree.converter()`.
  - Flexible: 
    Distance functions can be provided through a universal reference (forwarding reference).
    Also, filters are now movable and copyable. 
  
- **API BREAKING CHANGE**: Allow filtering on buckets in multimaps. Multimap filters have different functions
  and function signatures than normal `PhTree` filters. [#26](https://github.com/tzaeschke/phtree-cpp/issues/26)

### Fixed
- Fixed compiler warnings when compiling with Visual Studio 2019.
  [#74](https://github.com/tzaeschke/phtree-cpp/issues/74)
- Fixed cmake to work with Visual Studio 2019. Added tests and benchmarks to cmake.
  (benchmarks still do not work with VS at the moment).
  [#62](https://github.com/tzaeschke/phtree-cpp/issues/62)
- Fixed compilation problems and a memory leak when compiling with Visual Studio 2019.
  (also added `msan` support). [#64](https://github.com/tzaeschke/phtree-cpp/pull/64)

## [1.2.0] - 2022-04-14
### Changed
- Bugfix: FilterSphere was not working correctly. [#27](https://github.com/tzaeschke/phtree-cpp/issues/27)
- Potentially **BREAKING CHANGE**: Refactored API of all methods that accept callbacks and filters to
  accept universal/forwarding references.
  Also changed filters and callback to not require `const` methods. 
  [#22](https://github.com/tzaeschke/phtree-cpp/issues/22)
- Clean up iterator implementations. [#19](https://github.com/tzaeschke/phtree-cpp/issues/19)
- Make PhTree and PhTreeMultimap movable (move-assign/copy). [#18](https://github.com/tzaeschke/phtree-cpp/issues/18)
- Potentially **BREAKING CHANGE** when using `IsNodeValid()` in provided filters:
  Changed `bit_width_t` from `uin16_t` to `uint32_t`. This improves performance of 3D insert/emplace
  on small datasets by up to 15%. To avoid warnings that meant that the API of `FilterAABB` and `FilterSphere` 
  had to be changed to accept `uint32_t` instead of `int`. This may break some implementations.
  [#17](https://github.com/tzaeschke/phtree-cpp/pull/17)
- DIM>8 now uses custom b_plus_tree_map instead of std::map. This improves performance for all operations, e.g.
  window queries on large datasets are up to 4x faster. Benchmarks results can be found in the issue. 
  [#14](https://github.com/tzaeschke/phtree-cpp/issues/14)
- postfix/infix field moved from Node to Entry. This avoids indirections and improves performance of most by ~10%.
  operations by 5-15%.  [#11](https://github.com/tzaeschke/phtree-cpp/issues/11)
- Entries now use 'union' to store children.  [#9](https://github.com/tzaeschke/phtree-cpp/issues/9)
- Avoid unnecessary find() when removing a node. [#5](https://github.com/tzaeschke/phtree-cpp/issues/5)
- Avoid unnecessary key copy when inserting a node. [#4](https://github.com/tzaeschke/phtree-cpp/issues/4)
- for_each(callback, filter) was traversing too many nodes. [#2](https://github.com/tzaeschke/phtree-cpp/issues/2)
- Build improvements for bazel/cmake

## [1.1.1] - 2022-01-30
### Changed
- Replaced size() in filters with DIM [#26](https://github.com/improbable-eng/phtree-cpp/pull/26)

## [1.1.0] - 2022-01-25
### Added
- FilterSphere for filtering by sphere constraint (by ctbur) [#16](https://github.com/improbable-eng/phtree-cpp/pull/16)
- IEEE converter for 32bit float, see `distance.h` (by ctbur) [#18](https://github.com/improbable-eng/phtree-cpp/pull/18)

### Changed
- Performance improvement for updates and queries: removed use of `std::variant`. [#23](https://github.com/improbable-eng/phtree-cpp/pull/23)
- Fixed imports  `<climits>` -> `<limits>` (by ctbur) [#15](https://github.com/improbable-eng/phtree-cpp/pull/15)
- Cleaned up build scripts [#21](https://github.com/improbable-eng/phtree-cpp/pull/21)
- Fixed warnings: [#20](https://github.com/improbable-eng/phtree-cpp/pull/20)
  - "unused function argument" warnings
  - gcc/clang warnings
  - MSVC warnings
  - reserved identifier warnings (identifiers starting with `_`)
- typos in README.md [#22](https://github.com/improbable-eng/phtree-cpp/pull/22)

## [1.0.1] - 2021-05-06
### Changed
- replaced compilation flag `-fpermissive` with `-Werror`, and fixed all warnings/errors, see issue #10

## [1.0.0] - 2021-03-23
### Added
- API: `MultiMap`: A wrapper that makes PH-Tree behave as a multi-map.
- API: `erase(iterator)`
- API: `emplace_hint(iterator, ...)`
- API for `PhTreeF` and `PhTreeBoxF`: 32bit floating point options
- Support for custom key classes

### Changed
- BREAKING CHANGE: The query functions now require a query box as input (instead of a min/max point pair)
- BREAKING CHANGE: `phtree_box_d.h` has been removed, please use `phtree.h instead.
- BREAKING CHANGE: `phtree_d.h` has been removed, please use `phtree.h` instead.
- BREAKING CHANGE: Data converters (IEEE, Multiply, etc) are now structs i.o. functions/functors
- BREAKING CHANGE: `PhFilterNoOp` has been renamed to `FilterNoOp`
- BREAKING CHANGE: kNN queries now always require the distance function to be specified.
- BREAKING CHANGE: Preprocessors have been refactored and renamed to Converter/ScalarConverter
- Moved CI builds from Travis to GitHub actions

### Removed
- Nothing.

### Fixed
- GCC warnings from `-Wsign-compare` and `-Wsequence-point`.


## 0.1.0 - 2020-07-02
### Added
- Initial version.

### Changed
- Nothing.

### Removed
- Nothing.

### Fixed
- Nothing.


[Unreleased]: https://github.com/tzaeschke/phtree-cpp/compare/v1.6.1...HEAD
[1.6.1]: https://github.com/tzaeschke/phtree-cpp/compare/v1.6.0...v1.6.1
[1.6.0]: https://github.com/tzaeschke/phtree-cpp/compare/v1.5.0...v1.6.0
[1.5.0]: https://github.com/tzaeschke/phtree-cpp/compare/v1.4.0...v1.5.0
[1.4.0]: https://github.com/tzaeschke/phtree-cpp/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/tzaeschke/phtree-cpp/compare/v1.2.0...v1.3.0
[1.2.0]: https://github.com/improbable-eng/phtree-cpp/compare/v1.1.0...tzaeschke:phtree-cpp:v1.2.0
[1.1.1]: https://github.com/improbable-eng/phtree-cpp/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/improbable-eng/phtree-cpp/compare/v1.0.0...v1.1.0
[1.0.1]: https://github.com/improbable-eng/phtree-cpp/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/improbable-eng/phtree-cpp/compare/v0.1.0...v1.0.0
