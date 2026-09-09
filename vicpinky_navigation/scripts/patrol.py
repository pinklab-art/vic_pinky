import csv
import datetime
import time
 
import requests
 
# ─────────────────────────────────────────────────────────────
# 여기만 바꾼다
# ─────────────────────────────────────────────────────────────
ROBOT = "http://192.168.5.1:8080"
 
STATIONS = [
    {"name": "A", "x": 1.0, "y": 0.5, "yaw": 0.0},
    {"name": "B", "x": 2.5, "y": -1.0, "yaw": 1.57},
    {"name": "C", "x": 0.0, "y": 0.0, "yaw": 3.14},
]
 
ROUNDS = 2        # 몇 바퀴 돌 것인가
TIMEOUT = 90      # 한 구간 최대 대기 시간(초). 로봇이 못 갈 때의 탈출구
STAY = 3          # 스테이션에서 머무는 시간(초). 작업하는 시간을 흉내 낸 것
POLL = 0.5        # 도착했는지 물어보는 주기(초)
LOG_FILE = "patrol_log.csv"
# ─────────────────────────────────────────────────────────────
 
 
def is_navigating():
    """지금 주행 중인가?  (3번 자료의 curl .../api/nav/status 와 같은 요청)"""
    r = requests.get(ROBOT + "/api/nav/status", timeout=3)
    return r.json()["navigating"]
 
 
def send_goal(station):
    """목적지 보내기.  (3번 자료의 curl -X POST .../api/goal 과 같은 요청)"""
    r = requests.post(
        ROBOT + "/api/goal",
        timeout=3,
        json={"x": station["x"], "y": station["y"], "yaw": station["yaw"]},
    )
    return r.json().get("success", False)
 
 
def wait_until_idle(started_at):
    """
    도착할 때까지 기다린다.
 
    time.sleep(15) 처럼 시간을 정해 두고 기다리는 것이 아니라
    끝났는지 물어보고 기다린다. 이 차이가 스크립트와 관제를 가른다.
 
    보낸 직후에는 Nav2 가 아직 작업을 시작하지 않아 navigating 이 false 다.
    그대로 while 에 들어가면 출발도 전에 도착했다고 판단하므로 잠깐 쉬었다 시작한다.
    """
    time.sleep(1.0)
    while is_navigating() and (time.time() - started_at) < TIMEOUT:
        time.sleep(POLL)
    return (time.time() - started_at) < TIMEOUT   # 시간초과가 아니면 True
 
 
def main():
    log = csv.writer(open(LOG_FILE, "a", newline=""))
 
    for n in range(ROUNDS):
        for st in STATIONS:
            t0 = time.time()
            print("[%d바퀴] %s 으로 출발" % (n + 1, st["name"]))
 
            if not send_goal(st):
                print("   보내기 실패 — Nav2 가 떠 있는지 확인")
                continue
 
            in_time = wait_until_idle(t0)
            sec = round(time.time() - t0, 1)
            result = "도착" if in_time else "시간초과"
            print("   %s (%s초)" % (result, sec))
 
            log.writerow([
                datetime.datetime.now().isoformat(timespec="seconds"),
                st["name"], result, sec,
            ])
 
            time.sleep(STAY)
 
    print("순찰 종료 — 기록은 %s" % LOG_FILE)
 
 
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n중단됨 (로봇은 마지막 목적지까지 계속 간다. 세우려면 브라우저 Stop)")
