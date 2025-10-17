# Compiler et options
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2

# Dossiers
SRC_DIR := src
OBJ_DIR := build/obj
BIN_DIR := build/bin

# Nom de l'exécutable
TARGET := wallII

# Récupère tous les .cpp du dossier source
SRCS := $(wildcard $(SRC_DIR)/*.cpp)

# Transforme les .cpp en .o dans OBJ_DIR
OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# --------------------------------------------------
# Règle par défaut
all: $(BIN_DIR)/$(TARGET)

# Compilation de l'exécutable
$(BIN_DIR)/$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@

# Compilation des .o
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Création des dossiers si nécessaire
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Nettoyage
clean:
	rm -rf $(OBJ_DIR)/*.o $(BIN_DIR)/$(TARGET)

# Supprimer tout
distclean: clean
	rm -rf build

.PHONY: all clean distclean
