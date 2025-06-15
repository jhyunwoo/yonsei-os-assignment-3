CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra

all: vmsim

vmsim: vmsim.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -f vmsim
