RCS = $(wildcard *.cpp)
PROGS = $(patsubst %.cpp,%,$(SRCS))
OBJS = $(SRCS:.cpp=.o)
TEMPS = $(SRCS:.cpp=.txt)
OUT = controller
SRC = transmit.cpp receiver.cpp

CFLAGS = -lpthread `pkg-config --cflags --libs opencv`
LDFLAGS = `pkg-config --libs opencv`


all: controller


controller: controller.cpp
	g++ controller.cpp $(SRC) $(CFLAGS)  -o $(OUT)

clean:
	@rm -f $(PROGS) $(OBJS) $(TEMPS) $(OUT)
	@echo "Limpo!"
