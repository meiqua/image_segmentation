#include <pybind11/pybind11.h>
#include "np2mat/ndarray_converter.h"
#include "seg.h"
namespace py = pybind11;

PYBIND11_MODULE(cxxSeg_pybind, m) {
    NDArrayConverter::init_numpy();
}
