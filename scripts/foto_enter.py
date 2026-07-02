import cv2
import os
import threading
from datetime import datetime

RTSP_URL = "rtsp://localhost:8554/cam"
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset_images")

os.makedirs(SAVE_DIR, exist_ok=True)

latest_frame = None
running = True


def frame_reader():
    global latest_frame, running

    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print(f"Не удалось подключиться к {RTSP_URL}")
        running = False
        return

    print(f"Подключено: {RTSP_URL}")
    errors = 0

    while running:
        ret, frame = cap.read()
        if not ret:
            errors += 1
            if errors >= 5:
                print("Переподключение...")
                cap.release()
                cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
                errors = 0
            continue
        errors = 0
        latest_frame = frame

    cap.release()


reader = threading.Thread(target=frame_reader, daemon=True)
reader.start()

print("Ожидание потока", end="", flush=True)
import time
while latest_frame is None and running:
    print(".", end="", flush=True)
    time.sleep(0.3)
print()

if not running:
    exit(1)

count = 0
print("Готово. Нажимай Enter чтобы сохранить кадр, q + Enter чтобы выйти.\n")

while True:
    key = input()
    if key.strip().lower() == "q":
        break

    frame = latest_frame
    if frame is None:
        print("Кадр ещё не получен, попробуй ещё раз.")
        continue

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"frame_{timestamp}.jpg"
    path = os.path.join(SAVE_DIR, filename)
    cv2.imwrite(path, frame)
    count += 1
    print(f"[{count}] Сохранено: {path}")

running = False
print(f"\nЗавершено. Сохранено кадров: {count}")
print(f"Папка: {SAVE_DIR}")
