from config.database import engine  # 네 database.py에서 가져오기

try:
    with engine.connect() as conn:
        result = conn.exec_driver_sql("SELECT 1")
        print("연결 성공 ✅", result.scalar())  # 1이 나오면 정상
except Exception as e:
    print("연결 실패 ❌", e)
