OBJS = Parser.o algs.o main.o
CXX = g++
DEBUG = -g
CXXFLAGS = -Wall $(DEBUG) $(INCLUDES)
LXXFLAGS = -Wall $(DEBUG)
INCLUDES = -I.
LDLIBS = -L. -lbiscuit

race: $(OBJS) 
	$(CXX) $(LXXFLAGS) $(OBJS) -o race $(LDLIBS) 

$(OBJS): Parser.h algs.h Polygon.h coord.h

.PHONY: clean
clean:
	rm -f $(OBJS) core race

