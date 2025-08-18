import asyncio
import sys
from contextlib import suppress, asynccontextmanager
from pathlib import Path
from typing import Tuple, Dict, List
import uuid
import paho.mqtt.client as mqtt
import json
import numpy as np
from dotenv import load_dotenv
from fastapi import FastAPI, Request, HTTPException
from starlette.responses import HTMLResponse, StreamingResponse
from starlette.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from datetime import datetime
import mariadb


# mqtt 관련 설정
BROKER_HOST = "168.188.128.103"   # 예: "broker.example.com" 또는 "127.0.0.1"
BROKER_PORT = 1883              # TLS 사용 시 보통 8883
USERNAME = "test"
PASSWORD = "1234"
CLIENT_ID = f"esp32-{uuid.uuid4().hex[:8]}"


BASE_DIR = Path(__file__).resolve().parent.parent
print(BASE_DIR)
TEMPLATE_DIR = BASE_DIR.parent / "templates"
STATIC_DIR = BASE_DIR.parent / "static"

TOPICS = [
    ("/test", 0)      # 예시 토픽과 QoS. 필요에 맞게 변경         # 모든 토픽 구독 (테스트용)
]

RC_MESSAGES = {
    0: "SUCCESS",
    1: "REFUSED - incorrect protocol version",
    2: "REFUSED - invalid client identifier",
    3: "REFUSED - server unavailable",
    4: "REFUSED - bad username or password",
    5: "REFUSED - not authorised"
}

# 앵커 좌표 지정
ANCHORS: Dict[str, Tuple[float, float]] = {
    "ANC3": (7.05, 5.85),
    "ANC4" : (11.55, 3.00),
    "ANC6" : (7.05, 0.00),
    "ANC0": (0.45, 1.45)
}

@asynccontextmanager
async def lifespan(app: FastAPI):
    asyncio.create_task(start())
    yield
    await shutdown()


app = FastAPI(lifespan=lifespan)
subscribers = set()
templates = Jinja2Templates(directory=str(TEMPLATE_DIR))
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


load_dotenv()

def db_connect():
    try:
        conn = mariadb.connect(
            user="sooing",
            password="Qwer12345!",
            host="monet-lab.duckdns.org",
            port=13306,
            database="uwb"
        )
    except mariadb.Error as e:
        print(f"Error connecting to MariaDB Platform: {e}")
        sys.exit(1)
    return conn




@app.get("/")
async def get_map(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/tag", response_class=HTMLResponse)
async def get_tag_location(request: Request):
    return templates.TemplateResponse("tag.html", {"request": request})


#SSE 엔드포인트 (구독 전용)
@app.get("/sse/tag")
async def message_stream():
    # 클라이언트 1명당 큐를 생성해서 구독자 집합에 넣기
    q = asyncio.Queue(maxsize=32)
    subscribers.add(q)

    # 메시지 전달
    async def event_stream():
        try:
            while True:
                msg = await q.get()
                payload = json.dumps(msg, ensure_ascii=False)
                yield f"event: tag\ndata: {payload}\n\n" # 프론트 EventSoruce가 읽고 이벤트로 발생시킴
        finally:
            subscribers.discard(q)

    return StreamingResponse(event_stream(), media_type="text/event-stream")


# 큐에 메시지 넣기
async def publish_coordinates(x, y):
    msg = {"x": x, "y": y}
    for q in subscribers:
        if q.full():
            q.get_nowait() # 오래된 데이터 버림
        await q.put(msg)



#mqtt 연결
def on_connect(client, userdata, flags ,rc):
    msg = RC_MESSAGES.get(rc, "UNKNOWN")
    if rc == 0:
        print(f"MQTT 연결 성공: {msg}")
        for topic, qos in TOPICS:
            client.subscribe(topic, qos) # 토픽 구독
            print(f"구독 완료: {topic} (QoS={qos})")
    else:
        print(f"MQTT 연결 실패: {msg}")



# 메시지를 받았을 대 호출되는 콜백
def on_message(client, userdata, msg):

    global app_loop, dis0, dis3, dis4, dis6, fin_pos
    try:
        text = msg.payload.decode("utf-8")
        try:
            result = json.loads(text) #string을 json으로 변환
        except json.JSONDecodeError:
            print("JSONDecodeError")
            return

        if result is None:
            return
        print(result)


        data: List[Tuple[str, float]] = [] #(anc_id, distance, datetime)
        anchors: List[Tuple[float, float, float]] = [] # (x, y, dist)
        for anchor_id, (x, y) in ANCHORS.items():
            if anchor_id in result:
                dist = float(result[anchor_id]["dist"])
                anchors.append((x, y, dist))
                data.append((anchor_id, dist))


        trial_result = trilaterate(anchors)
        print(trial_result)

        x, y, result_time = trilaterate(anchors)
        fin_pos = f"({x:.2f}, {y:.2f})"



        dist_map = {k: float(v["dist"]) for k, v in result.items() if "dist" in v}
        dis0 = dist_map.get("ANC0")
        dis3 = dist_map.get("ANC3")
        dis4 = dist_map.get("ANC4")
        dis6 = dist_map.get("ANC6")


        def fmt(v):
            return "NA" if v is None else f"{v:.2f}"

        print(f"{fin_pos} ANC0={fmt(dis0)} ANC3={fmt(dis3)} ANC4={fmt(dis4)} ANC6={fmt(dis6)}")

        #db에 저장
        insert_db(result_time, round(dis0, 2), round(dis3, 2), round(dis4, 2), round(dis6, 2), fin_pos)


        # 프론트엔드로 전달
        if app_loop and x >= 0 and y >= 0:
            asyncio.run_coroutine_threadsafe(
                publish_coordinates(x, y), app_loop
            )

        # db 저장 - 앵커 아이디, 좌표


    except UnicodeDecodeError:
        text = repr(msg.payload)
        print(text)



def on_disconnect(client, userdata, rc):
    msg = RC_MESSAGES.get(rc, "UNKNOWN")
    print(f"MQTT 연결 종료: {msg}")



# 사변측량
def trilaterate(anchors: List[Tuple[float, float, float]]) -> Tuple[float, float, datetime]:
    dist = [] # 앵커별 거리 저장
    anchor_xy: List[Tuple[float, float]] = [] # 앵커 좌표 저장
    x = 0.0
    y = 0.0
    result: Tuple[float, float, datetime] = (x, y, datetime.now())

    # 거리가 3개 이하면 계산 불가
    if len(anchors) < 3:
        return -1, -1, datetime.now()

    for i in range(len(anchors)):
        anchor_xy.append((anchors[i][0], anchors[i][1]))
        dist.append(anchors[i][2])

    x1, y1, d1 = anchors[0]

    A = []
    B = []
    # 위치 계산
    for i in range(1, len(anchor_xy)):
        x2, y2, d2 = anchors[i]
        A.append([2*(x2-x1), 2*(y2-y1)])
        B.append(d1**2 - d2**2 + x2**2 - x1**2 + y2**2 - y1**2)

    A = np.array(A)
    B = np.array(B)

    try:
        result, residuals, rank, s = np.linalg.lstsq(A, B, rcond=None)
        x, y = float(result[0]), float(result[1])

        x = round(x, 2)
        y = round(y, 2)
        return x, y, datetime.now()
    except np.linalg.LinAlgError:
        return -1, -1, datetime.now()


def insert_db(date, anc0, anc3, anc4, anc6, fin_pos):
    conn = db_connect()
    cursor = conn.cursor()

    sql = """
          INSERT INTO tag (date, anc0, anc3, anc4, anc6, fin_pos)
          VALUES (%s, %s, %s, %s, %s, %s) \
          """

    if isinstance(date, str):
        cursor.execute(sql, (date, anc0, anc3, anc4, anc6, fin_pos))
    else:
        cursor.execute(sql, (date, anc0, anc3, anc4, anc6, fin_pos))

    conn.commit()
    conn.close()




async def start():
    global app_loop, mqtt_client
    app_loop = asyncio.get_running_loop()



    mqtt_client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
    if USERNAME:
        mqtt_client.username_pw_set(USERNAME, PASSWORD)

    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.on_disconnect = on_disconnect

    mqtt_client.connect_async(BROKER_HOST, BROKER_PORT, keepalive=30)
    mqtt_client.loop_start()  # 비동기 루프 스레드 시작
    print("[APP] startup OK")


async def shutdown():
    global mqtt_client
    if mqtt_client is not None:
        mqtt_client.loop_stop()
        with suppress(Exception):
            mqtt_client.disconnect()
    print("[APP] shutdown OK")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("main:app", host="127.0.0.1", port=8000)
