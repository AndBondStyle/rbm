#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

REPO_URL="https://github.com/AndBondStyle/rbm"
TARGET_DIR="$HOME/robomarvel"
NEEDS_REBOOT=false

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

if [ "$EUID" -eq 0 ]; then 
    log_error "Не запускайте скрипт от пользователя root!"
    log_error "Используйте обычного пользователя с правами sudo"
    exit 1
fi

check_command() {
    command -v $1 >/dev/null 2>&1
}

# 1. Установка Docker
install_docker() {
    log_info "Проверка установки Docker..."
    if check_command docker; then
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

# 2. Установка udev правил для PlatformIO
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

# 3. Настройка /boot/firmware/config.txt
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

# 4. Настройка PSU_MAX_CURRENT
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

# 5. Клонирование репозитория
clone_repository() {
    log_info "Проверка репозитория..."
    
    if [ -d "$TARGET_DIR/.git" ]; then
        log_info "Репо уже склонирован в $TARGET_DIR"
        log_info "Обновление репозитория..."
        cd "$TARGET_DIR"
        git pull
        return 0
    fi
    
    log_info "Клонирование репозитория..."
    git clone "$REPO_URL" "$TARGET_DIR"
    cd "$TARGET_DIR"
    log_info "Репо успешно склонирован"
}

# 6. Запуск docker compose
run_docker_compose() {
    log_info "Запуск Docker Compose..."
    cd "$TARGET_DIR"
    
    log_info "Загрузка Docker образов..."
    sudo docker compose pull
    log_info "Запуск Docker контейнеров..."
    sudo docker compose up -d
    log_info "Проверка запущенных контейнеров..."
    sudo docker compose ps
}

main() {
    log_info "Начало настройки Raspberry Pi"

    log_info "Обновление списка пакетов..."
    sudo apt-get update
    log_info "Установка необходимых утилит..."
    sudo apt-get install -y git curl udev
    
    install_docker
    install_platformio_udev
    configure_config_txt
    configure_psu_current
    clone_repository
    run_docker_compose
    
    echo ""
    log_info "=== НАСТРОЙКА ЗАВЕРШЕНА ==="
    
    if [ "$NEEDS_REBOOT" = true ]; then
        log_warn "ТРЕБУЕТСЯ ПЕРЕЗАГРУЗКА СИСТЕМЫ"
        echo ""
        
        read -p "Выполнить перезагрузку сейчас? (y/N): " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            log_info "Перезагрузка системы..."
            sudo reboot
        else
            log_info "Перезагрузите систему позже командой: sudo reboot"
        fi
    else
        log_info "Перезагрузка не требуется"
    fi
    
    log_info "Настройка завершена успешно!"
}

# Обработка ошибок
trap 'log_error "Скрипт прерван!"; exit 1' INT TERM

# Запуск основной функции
main "$@"
