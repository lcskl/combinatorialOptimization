DIRS := . MinCost GEOM 

SOURCES := $(foreach dir, $(DIRS), $(wildcard $(dir)/*.cpp))
OBJS := $(patsubst %.cpp, %.o, $(SOURCES))

CFLAGS := -O5 -D_NDEBUG
CXX ?= c++
LIBS := 
INCLUDES := 
LIBDIR := 

# Add librt if the target platform is not Darwin (OS X)
ifneq ($(shell uname -s),Darwin)
    LIBS += -lrt
endif
 
all: blossom5.a

blossom5.a: ${OBJS}
	ar crv blossom5.a ${OBJS}

.cpp.o:
	$(CXX) $(CFLAGS) ${INCLUDES} $< -c -o $@

clean:
	rm -f ${OBJS} blossom5.a
