#!/bin/bash

# 🔹 Konfiguration (Passe diese Werte an!)
RASPBERRY_PI_USER="pi"               
RASPBERRY_PI_HOST="10.192.58.70"  # IP-Adresse des Raspberry Pi
RASPBERRY_PI_PATH="/home/pi/2024-2025-Robotics-cup/src/robonav/build" # Zielpfad auf dem Raspberry Pi
LOCAL_WS_PATH="$HOME/buildforrpi/2024-2025-Robotics-cup/src/robonav"  # Dein CMake-Projekt auf dem Laptop
BUILD_PATH="$LOCAL_WS_PATH/build"   # CMake Build-Verzeichnis
EXECUTABLE_NAME="main_node"         # Name der Binärdatei
GIT_BRANCH="ferdinand_rpi"          # Branch, den du pullen willst

echo "🔄 Hole neueste Änderungen von GitHub..."
cd "$LOCAL_WS_PATH" || exit
git pull origin "$GIT_BRANCH"

echo "🚀 Starte schnellen Build mit CMake..."
mkdir -p "$BUILD_PATH"
cd "$BUILD_PATH" || exit
cmake .. -DCMAKE_BUILD_TYPE=Debug  # Falls du Release willst, ändere zu Release
make -j$(nproc)                    # Nutzt alle verfügbaren CPU-Kerne

# Prüfen, ob die Datei erfolgreich erstellt wurde
BIN_PATH="$BUILD_PATH/$EXECUTABLE_NAME"
if [ ! -f "$BIN_PATH" ]; then
    echo "❌ Fehler: Build fehlgeschlagen! Datei $EXECUTABLE_NAME nicht gefunden."
    exit 1
fi

echo "📤 Sende Datei an den Raspberry Pi..."
scp "$BIN_PATH" "$RASPBERRY_PI_USER@$RASPBERRY_PI_HOST:$RASPBERRY_PI_PATH"

echo "🔄 Starte Programm auf dem Raspberry Pi..."
ssh "$RASPBERRY_PI_USER@$RASPBERRY_PI_HOST" << EOF
    cd $RASPBERRY_PI_PATH
    chmod +x "$EXECUTABLE_NAME"
    ./"$EXECUTABLE_NAME"
EOF

echo "✅ Deployment abgeschlossen!"
