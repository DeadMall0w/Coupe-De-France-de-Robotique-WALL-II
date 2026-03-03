# ======================================================
#      ⚙️ Makefile — Test Visuel LIDAR
# ======================================================
# Usage :
#   make              → compile le test
#   make run          → compile et lance (position par défaut : 50, 100, 0°)
#   make run ARGS="150 100 90"  → lance avec position custom
#   make serve        → lance un serveur HTTP pour la visualisation
#   make clean        → supprime les fichiers compilés

# ---- Chemins relatifs vers Dev/ ----
DEV_DIR := ../../Dev
SRC_DIR := $(DEV_DIR)/src
INC_DIR := $(DEV_DIR)/includes

# ---- Compilateur ----
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra \
            -I$(DEV_DIR)/lib/include

# ---- Librairie LIDAR ----
LIDAR_LIB := $(DEV_DIR)/lib/bin/libsl_lidar_sdk.a

# ---- Fichiers source ----
# On réutilise les mêmes fichiers que Dev, sauf main.cpp et vision.cpp (on a notre propre main)
DEV_SRCS := $(SRC_DIR)/board.cpp \
            $(SRC_DIR)/Slamtec.cpp \
            $(SRC_DIR)/LidarProcessing.cpp

TEST_SRC := test_lidar_visual.cpp

# ---- Fichiers objets ----
OBJ_DIR := build
DEV_OBJS := $(OBJ_DIR)/board.o $(OBJ_DIR)/Slamtec.o $(OBJ_DIR)/LidarProcessing.o
TEST_OBJ := $(OBJ_DIR)/test_lidar_visual.o
ALL_OBJS := $(DEV_OBJS) $(TEST_OBJ)

# ---- Cible ----
TARGET := $(OBJ_DIR)/test_lidar_visual

# Arguments par défaut pour run
ARGS ?= 50 100 0

# ======================================================
#        🏁 Compilation
# ======================================================
all: $(TARGET)

$(TARGET): $(ALL_OBJS) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(ALL_OBJS) $(LIDAR_LIB) -o $@ -lpthread

# Objets depuis Dev/src/
$(OBJ_DIR)/board.o: $(SRC_DIR)/board.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/Slamtec.o: $(SRC_DIR)/Slamtec.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/LidarProcessing.o: $(SRC_DIR)/LidarProcessing.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Objet du test
$(OBJ_DIR)/test_lidar_visual.o: $(TEST_SRC) | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR):
	mkdir -p $@

# ======================================================
#        🚀 Lancement
# ======================================================
run: $(TARGET)
	./$(TARGET) $(ARGS)

# Lance un serveur HTTP local pour visualiser le HTML
# Ouvre http://localhost:8080/lidar_viewer.html dans ton navigateur
serve:
	@echo "🌐 Serveur HTTP sur http://localhost:8080/lidar_viewer.html"
	@echo "   (Ctrl+C pour arrêter)"
	python3 -m http.server 8089

# ======================================================
#        🧹 Nettoyage
# ======================================================
clean:
	rm -rf build lidar_debug.json

.PHONY: all run serve clean
