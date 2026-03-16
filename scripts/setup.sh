#!/bin/bash

REPO_DIR="$HOME/robomarvel"
REPO_URL="https://git@git.sourcecraft.dev/robomarvel/rbm-runtime.git"
SETUP_DIR="$HOME/.setup"
VENV_DIR="$SETUP_DIR/venv"
TEST_SCRIPT_URL="https://raw.githubusercontent.com/AndBondStyle/rbm/refs/heads/master/scripts/test.py"
TEST_SCRIPT_PATH="$SETUP_DIR/test.py"
REQUIREMENTS=("pyserial" "nicegui")
SERVICE_NAME="rbm-web-tests"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

NEEDS_REBOOT=false

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

if [ "$EUID" -eq 0 ]; then 
    log_error "Не запускайте скрипт от пользователя root!"
    log_error "Используйте обычного пользователя с правами sudo"
    exit 1
fi

install_apt_packages() {
    log_info "Установка APT пакетов..."
    sudo apt-get update
    sudo apt-get install -y git curl udev
}

install_docker() {
    log_info "Проверка установки Docker..."
    if command -v docker >/dev/null 2>&1; then
        log_info "Docker уже установлен"
        return 0
    fi
    
    log_info "Установка Docker..."
    curl -fsSL https://get.docker.com | sh    
    sudo usermod -aG docker $USER

    log_info "Docker успешно установлен"
    log_info "Пользователь $USER добавлен в группу docker"
    log_warn "Для обновления групп запустите: exec sudo su -l \$USER"
}

install_platformio_udev() {
    log_info "Проверка udev правил для PlatformIO..."
    UDEV_RULES_FILE="/etc/udev/rules.d/99-platformio-udev.rules"
    UDEV_RULES_URL="https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules"
    
    if [ -f "$UDEV_RULES_FILE" ] && \
       curl -fsSL "$UDEV_RULES_URL" | diff -q "$UDEV_RULES_FILE" - >/dev/null 2>&1; then
        log_info "Актуальные udev правила для PlatformIO уже установлены"
        return 0
    fi
    
    log_info "Загрузка и установка udev правил для PlatformIO..."
    curl -fsSL "$UDEV_RULES_URL" | sudo tee "$UDEV_RULES_FILE" > /dev/null
    sudo chown root:root "$UDEV_RULES_FILE"
    sudo chmod 0644 "$UDEV_RULES_FILE"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    log_info "Udev правила для PlatformIO успешно загружены и установлены"
}

configure_config_txt() {
    log_info "Проверка конфигурации /boot/firmware/config.txt..."    
    CONFIG_FILE="/boot/firmware/config.txt"
    MODIFIED=false
    
    # Проверка и отключение dtparam=audio=on
    if grep -q "^dtparam=audio=on" "$CONFIG_FILE"; then
        log_info "Отключение dtparam=audio=on..."
        sudo sed -i 's/^dtparam=audio=on/#dtparam=audio=on/' "$CONFIG_FILE"
        MODIFIED=true
    fi
    
    # Проверка и добавление noaudio к vc4-kms-v3d
    # Ищем строку с vc4-kms-v3d без noaudio
    if grep -q "^dtoverlay=vc4-kms-v3d" "$CONFIG_FILE" && \
       ! grep -q "^dtoverlay=vc4-kms-v3d,noaudio" "$CONFIG_FILE" && \
       ! grep -q "^dtoverlay=vc4-kms-v3d,.*,noaudio" "$CONFIG_FILE"; then
        log_info "Добавление noaudio к vc4-kms-v3d..."
        sudo sed -i 's/^dtoverlay=vc4-kms-v3d/dtoverlay=vc4-kms-v3d,noaudio/' "$CONFIG_FILE"
        MODIFIED=true
    fi
    
    # Проверка и добавление dtparam=uart0=on
    if ! grep -q "^dtparam=uart0=on" "$CONFIG_FILE"; then
        log_info "Добавление dtparam=uart0=on..."
        echo "dtparam=uart0=on" | sudo tee -a "$CONFIG_FILE" > /dev/null
        MODIFIED=true
    fi
    
    # Проверка и добавление dtparam=i2s=on
    if ! grep -q "^dtparam=i2s=on" "$CONFIG_FILE"; then
        log_info "Добавление dtparam=i2s=on..."
        echo "dtparam=i2s=on" | sudo tee -a "$CONFIG_FILE" > /dev/null
        MODIFIED=true
    fi
    
    # Проверка и добавление dtoverlay=googlevoicehat-soundcard
    if ! grep -q "^dtoverlay=googlevoicehat-soundcard" "$CONFIG_FILE"; then
        log_info "Добавление dtoverlay=googlevoicehat-soundcard..."
        echo "dtoverlay=googlevoicehat-soundcard" | sudo tee -a "$CONFIG_FILE" > /dev/null
        MODIFIED=true
    fi
    
    if [ "$MODIFIED" = true ]; then
        log_warn "Файл config.txt был изменен"
        NEEDS_REBOOT=true
    else
        log_info "Файл config.txt уже настроен"
    fi
}

configure_psu_current() {
    log_info "Проверка настройки PSU_MAX_CURRENT..."
    CURRENT_CONFIG=$(sudo rpi-eeprom-config)
    
    if echo "$CURRENT_CONFIG" | grep -q "PSU_MAX_CURRENT=5000"; then
        log_info "PSU_MAX_CURRENT уже настроен на 5000 мА"
        return 0
    fi
    
    log_info "Настройка PSU_MAX_CURRENT=5000..."    
    TMP_CONFIG=$(mktemp)
    echo "$CURRENT_CONFIG" > "$TMP_CONFIG"

    if echo "$CURRENT_CONFIG" | grep -q "PSU_MAX_CURRENT="; then
        sed -i 's/PSU_MAX_CURRENT=.*/PSU_MAX_CURRENT=5000/' "$TMP_CONFIG"
    else
        echo "PSU_MAX_CURRENT=5000" >> "$TMP_CONFIG"
    fi

    sudo rpi-eeprom-config --apply "$TMP_CONFIG"
    rm "$TMP_CONFIG"
    
    log_warn "Конфигурация EEPROM обновлена"
    NEEDS_REBOOT=true
}

clone_repository() {
    log_info "Проверка репозитория..."
    
    if [ -d "$REPO_DIR/.git" ]; then
        log_info "Репо уже склонирован в $REPO_DIR"
        log_info "Обновление репозитория..."
        cd "$REPO_DIR"
        git pull
        return 0
    fi
    
    log_info "Клонирование репозитория..."
    git clone "$REPO_URL" "$REPO_DIR"
    cd "$REPO_DIR"

    log_info "Добавление флага autostart..."
    touch "$REPO_DIR/.autostart"
}

run_docker_compose() {
    log_info "Запуск Docker Compose..."
    cd "$REPO_DIR"
    
    log_info "Загрузка Docker образов..."
    sudo docker compose pull
    log_info "Запуск Docker контейнеров..."
    sudo docker compose up -d --no-build
    sleep 10
    log_info "Проверка запущенных контейнеров..."
    sudo docker compose ps
}

setup_web_tests_service() {
    mkdir -p "$SETUP_DIR"

    if [[ ! -d "$VENV_DIR" ]]; then
        log_info "Creating virtual environment in $VENV_DIR"
        python3 -m venv "$VENV_DIR"
    else
        log_info "Virtual environment already exists, skipping creation"
    fi

    local pkg
    for pkg in "${REQUIREMENTS[@]}"; do
        if ! "$VENV_DIR/bin/pip" list --format=freeze | grep -q "^${pkg}=="; then
            log_info "Installing $pkg"
            "$VENV_DIR/bin/pip" install "$pkg"
        else
            log_info "$pkg already installed, skipping"
        fi
    done

    log_info "Ensuring $TEST_SCRIPT_PATH is up to date"
    if ! curl -fsSL "$TEST_SCRIPT_URL" -o "$TEST_SCRIPT_PATH"; then
        log_error "Failed to download $TEST_SCRIPT_URL"
    fi
    chmod +x "$TEST_SCRIPT_PATH"

    log_info "Setting up systemd service $SERVICE_NAME"
    local service_unit="[Unit]
Description=RBM Web Tests
After=docker.service
Requires=docker.service

[Service]
Type=simple
User=$USER
WorkingDirectory=$SETUP_DIR
ExecStart=$VENV_DIR/bin/python $TEST_SCRIPT_PATH
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
"

    if [[ -f "$SERVICE_FILE" ]]; then
        local existing
        existing=$(sudo cat "$SERVICE_FILE")
        if [[ "$existing" == "$service_unit" ]]; then
            log_info "Service file already up to date"
        else
            log_info "Updating existing service file"
            echo "$service_unit" | sudo tee "$SERVICE_FILE" >/dev/null
            sudo systemctl daemon-reload
        fi
    else
        log_info "Creating new service file"
        echo "$service_unit" | sudo tee "$SERVICE_FILE" >/dev/null
        sudo systemctl daemon-reload
    fi

    if ! sudo systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
        log_info "Enabling service to start on boot"
        sudo systemctl enable "$SERVICE_NAME"
    else
        log_info "Service already enabled"
    fi

    if ! sudo systemctl is-active --quiet "$SERVICE_NAME"; then
        log_info "Starting service now"
        sudo systemctl start "$SERVICE_NAME"
    else
        log_info "Service is already running"
    fi
}

main() {
    log_info "=== НАЧАЛО НАСТРОЙКИ RASPBERRY ==="

    install_apt_packages
    install_docker
    install_platformio_udev
    configure_config_txt
    configure_psu_current
    clone_repository
    run_docker_compose
    setup_web_tests_service
    
    log_info "=== НАСТРОЙКА ЗАВЕРШЕНА ==="
    if [ "$NEEDS_REBOOT" = true ]; then
        log_warn "Запланирована перезагрузка через 1 минуту"
        log_warn "Для отмены перезагрузки: sudo shutdown -c"
        sudo shutdown -r +1
    fi
}

trap 'log_error "Скрипт прерван!"; exit 1' INT TERM
main "$@"
