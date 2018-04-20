CC=clang++
SRC_DIR := src
OBJ_DIR := obj
SRC_FILES := $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))
LDFLAGS := -lglfw -ldl -lGL
CPPFLAGS := --std=c++14
CXXFLAGS := -Wall -O3 -Iinclude

cg2: $(OBJ_FILES)
	$(CC) $(LDFLAGS) -o $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	rm obj/*.o cg2
