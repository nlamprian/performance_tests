# performance_tests
Performance tests on combinations of ROS publishers and subscribers written in C++ and Python.

![sample](data/sample.png)

# Usage
Run the provided launch files to start the tests. Each launch file starts a publisher, a subscriber, a reconfigure and a multiplot plugin. You are able to change the publisher frequency and examine the results on the multiplot instance.

| Launch File | Publisher | Subscriber |
|:----------- |:---------:|:----------:|
| `performace_tests_cpp_cpp` |  C++   |  C++   |
| `performace_tests_cpp_py`  |  C++   | Python |
| `performace_tests_py_cpp`  | Python |  C++   |
| `performace_tests_py_py`   | Python | Python |

# Results

The following table summarizes the behaviours observed while testing the 4 combinations of publishers and subscribers.

| Publisher | Subscriber | Observations |
|:---------:|:----------:|:------------ |
|  C++   |  C++   | The frequencies of the two nodes match. They follow the desired frequency, but there is an increasing discrepancy as the frequency goes higher. |
|  C++   | Python | The frequencies of the two nodes match up to 2kHz. Increasing the publisher frequency beyond that point leaves the subscriber frequency unaffected.|
| Python |  C++   | The frequencies of the two nodes match, but they are throttled by the publisher at 2kHz. |
| Python | Python | The frequencies of the two nodes match up to around 2kHz. Pushing the desired frequency further, leaves the subscriber below 2kHz, and the publisher maxes out just above 2kHz. |

Tested on a PC with an ASUS 990FX mobo, an AMD FX-9370 CPU and ROS Kinetic.

# Roadmap
Add tests with nodelets to investigate how the resulting frequencies grow relative to the ones of the C++ nodes.