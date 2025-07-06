CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -fsanitize=address
LDFLAGS = -fsanitize=address
LIBS =

SRCS = HW5.cpp Graph.cpp Vertex.cpp Edge.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = hw5

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f $(OBJS) $(TARGET)
