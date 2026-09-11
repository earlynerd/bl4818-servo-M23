# Host protocol tests

Compile the real library against the minimal test-only Stream adapter:

```sh
g++ -std=c++11 -Wall -Wextra -Werror -Itests/arduino_ring -Ilibraries/BL4818Ring/src tests/arduino_ring/test_ring.cpp libraries/BL4818Ring/src/BL4818Ring.cpp -o build/test_arduino_ring
./build/test_arduino_ring
```

MSVC also works from a Developer Command Prompt:

```bat
cl /nologo /EHsc /std:c++14 /Itests/arduino_ring /Ilibraries/BL4818Ring/src tests/arduino_ring/test_ring.cpp libraries/BL4818Ring/src/BL4818Ring.cpp /Fobuild/ /Febuild/test_arduino_ring.exe
build\test_arduino_ring.exe
```

Covers CRC-CCITT check value, wire encoding, signed status decoding, echoed
commands, mismatched replies, invalid arguments, rejection detail, corrupted
and nested frames, enumeration transitions, stale input, timer wraparound,
timeout, short writes and unchanged status on failure. This is simulated
transport testing; compile the example against Arduino-Pico separately and
qualify motion and wiring on the bench.
