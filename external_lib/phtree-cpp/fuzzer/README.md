# Fuzzing


Requirements:
 * `clang`.
 * libFuzzer: https://github.com/google/fuzzing/blob/master/tutorial/libFuzzerTutorial.md 

Compile one of:
* `clang++ -g -std=c++17 -fsanitize=fuzzer fuzzer/b_plus_multimap_fuzzer.cc -I.`
* `clang++ -g -std=c++17 -fsanitize=fuzzer fuzzer/b_plus_map_fuzzer.cc -I.`
* `clang++ -g -std=c++17 -fsanitize=fuzzer fuzzer/b_plus_hash_map_fuzzer.cc -I.`

Execute:
* `./a.out`
* `./a.out -minimize_crash=1 -runs=10000 /tmp/tmp.b521097a4f49`
* `./a.out /tmp/tmp.12345678/artifacts/minimized-from-185ecf42f208c2a7736a98ba0403f31868bcb681`

To give an artifact path:
* `-artifact_prefix=/home/my-name/tmp/fuzz/artifacts/`

## Bazel

Fuzzing with bazel is possible but is currently disabled because it breaks `bazel build ...`.

* We would need to set `clang`/`clang++` as compiler (`gcc` would not work anymore)
* We would need to solve the problem that `bazel build ...` fails unless `-fsanitize=fuzzer` is set

### Using a simple executable
Uncomment build rules in BUILD file, then:
`CC=clang bazel run //fuzzer:b_plus_multimap_fuzzer --config=fuzz`

### Using the bazel cc_fuzz_test
https://github.com/bazelbuild/rules_fuzzing/blob/master/docs/guide.md
