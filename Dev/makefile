# ======================================================
#      ⚙️ Makefile — WALL-II (Official / Debug)
# ======================================================

# Compilateur et options par défaut
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra \
            -I$(PWD)/lib/include \
            $(shell pkg-config --cflags opencv4)

# Flags de linkage OpenCV
LDFLAGS := $(shell pkg-config --libs opencv4)

# Répertoires source et binaires
SRC_DIR := src
BIN_DIR := build/bin

# Nom de l'exécutable
TARGET := wallII

# Tous les fichiers source
# Récupère les .cpp dans src/ ET dans src/strategies/
SRCS := $(wildcard $(SRC_DIR)/*.cpp) $(wildcard $(SRC_DIR)/strategies/*.cpp)


# Objets
OFFICIAL_OBJ_DIR := build/obj/official
OFFICIAL_OBJS := $(patsubst $(SRC_DIR)/%.cpp,$(OFFICIAL_OBJ_DIR)/%.o,$(SRCS))

# Librairie LIDAR
LIDAR_LIB := $(PWD)/lib/bin/libsl_lidar_sdk.a

# ======================================================
#        🏁 Version officielle (Tournoi)
# ======================================================
official: CXXFLAGS += -O2
official: $(BIN_DIR)/$(TARGET)


# ======================================================
#        🙈 Version sans Caméra
# ======================================================
nocam: CXXFLAGS += -O2 -DNO_CAM
nocam: $(BIN_DIR)/$(TARGET)



$(BIN_DIR)/$(TARGET): $(OFFICIAL_OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OFFICIAL_OBJS) $(LIDAR_LIB) $(LDFLAGS) -o $@

# Modifie la règle de création des objets pour préserver la hiérarchie
OFFICIAL_OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(OFFICIAL_OBJ_DIR)/%.o)

# Règle de compilation générique
$(OFFICIAL_OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)  # Crée dynamiquement le sous-dossier (ex: build/obj/official/strategies/)
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
