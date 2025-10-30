# ======================================================
#      ⚙️ Makefile — WALL-II (Official / Debug)
# ======================================================

# Compilateur et options par défaut
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra \
            -I/home/rapha/testLidar/rplidar_sdk/sdk/include \
            -I/home/rapha/testLidar/rplidar_sdk/sdk/src \
            -I/home/rapha/testLidar/rplidar_sdk/sdk/src/hal

# Répertoires source et binaires
SRC_DIR := src
BIN_DIR := build/bin

# Nom de l'exécutable
TARGET := wallII

# Tous les fichiers source
SRCS := $(wildcard $(SRC_DIR)/*.cpp)

# Objets
OFFICIAL_OBJ_DIR := build/obj/official
OFFICIAL_OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OFFICIAL_OBJ_DIR)/%.o,$(SRCS))

# Librairie LIDAR
LIDAR_LIB := /home/rapha/testLidar/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a

# ======================================================
#        🏁 Version officielle (Tournoi)
# ======================================================
official: CXXFLAGS += -O2
official: $(BIN_DIR)/$(TARGET)

$(BIN_DIR)/$(TARGET): $(OFFICIAL_OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OFFICIAL_OBJS) $(LIDAR_LIB) -o $@

$(OFFICIAL_OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OFFICIAL_OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# ======================================================
#        🧹 Création des dossiers
# ======================================================
$(OFFICIAL_OBJ_DIR):
	mkdir -p $@

$(BIN_DIR):
	mkdir -p $@

# ======================================================
#        🧹 Nettoyage
# ======================================================
clean:
	rm -rf build/obj/*

distclean:
	rm -rf build

.PHONY: official clean distclean
