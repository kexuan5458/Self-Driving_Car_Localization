CXX = g++
CXXFLAGS = -O3 -Wall -Wextra -shared -std=c++14 -fPIC
PYBIND_INCLUDE = `python3 -m pybind11 --includes`
PYINCLUDE = `python3-config --includes`
TARGET = _localization`python3-config --extension-suffix`

MKL_INC = -I/usr/include/mkl
MKL_LIB = -L/usr/lib/x86_64-linux-gnu -lblas
IBLAS = -lblas
PYFLAGS = -lpython3.8

.PHONY: main, test, clean

main: 
	$(CXX) $(CXXFLAGS) $(PYINCLUDE) -I./ $(PYBIND_INCLUDE) localizer_node_py.cpp -o $(TARGET) $(PYFLAGS) $(IBLAS)

test: main
	  python3 -m pytest -v -s
clean:
	  rm -rf *.so __pycache__ .pytest*