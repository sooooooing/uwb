#include "dw3000.h"
#include <WiFi.h> //wifi 연결
#include <PubSubClient.h> //mqtt
#include "ArduinoJson.h" //json 관련
#include <math.h>

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4
#define RNG_DELAY_MS 1000
// #define TX_ANT_DLY 16385
// #define RX_ANT_DLY 16385
// #define ALL_MSG_COMMON_LEN 10
// #define ALL_MSG_SN_IDX 2
// #define RESP_MSG_POLL_RX_TS_IDX 10
// #define RESP_MSG_RESP_TX_TS_IDX 14
// #define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 10000


#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2  // 시퀀스 번호 시작 위치 (2바이트 사용)
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define RANDOM_DELAY_IDX 18
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000
#define ANCHOR_ID "ANC1"



#define MAX_ANCHORS 4


String get_wifi_status(int status){
    switch(status){
        case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
        case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
        case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
        case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
        case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
        case WL_CONNECTED:
        return "WL_CONNECTED";
        case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    }
}


/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
       5,                /* Channel number. */
    DWT_PLEN_512,     /* Preamble length. Used in TX only. */
    DWT_PAC32,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_850K,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (513 + 8 - 32),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

//앵커테이블 구조체 
struct Anchor_table {
  char anchor_id[5];
  double distance;
  uint8_t seq;
};


static uint8_t frame_seq_nb = 0;


static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'G', '0', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'N', 'C', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t rx_buffer[24];
static uint32_t status_reg = 0;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static double tof;
static double distance;


static double distances[MAX_ANCHORS]; // 각 앵커별 거리 저장
static char anchor_table[MAX_ANCHORS][5]; //앵커 아이디 저장
//static char anchor_table[MAX_ANCHORS][5] = {"ANC0", "ANC3", "ANC4", "ANC6"};
static bool received[MAX_ANCHORS] = {false}; 

static char anchor_id[5] = {0}; 
static bool is_used[MAX_ANCHORS] = {false}; 


//wifi
const char* ssid = "monet2g";
const char* password = "monet1234";

//mqtt
const char* mqtt_broker = "168.188.128.103";
const char* topic = "/test";
const char* mqtt_username = "test";
const char* mqtt_password = "1234";
const int mqtt_port = 1883;


WiFiClient espClient;
PubSubClient client(espClient);


extern dwt_txconfig_t txconfig_options;



void setup()
{
  Serial.begin(115200);  
  delay(1000);
  int status = WL_IDLE_STATUS;
  Serial.println(get_wifi_status(status));


  // TX 전력 최대 설정
  txconfig_options.PGdly = 0x34;
  txconfig_options.power = 0xFFFFFFFF;  // 최대 전력 설정
  txconfig_options.PGcount = 0x0;

  //와이파이 연결부분
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");


  while(WiFi.status() != WL_CONNECTED) {
    delay(100);
    status = WiFi.status();
    Serial.println(get_wifi_status(status));
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());


  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("Public EMQX MQTT broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }



  Serial.println("setup시작");
  UART_init();

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS); //송신 후 얼마나 있다가 수신을 시작할지 설정
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS); // 응답 수신을 기다릴 최대 시간 설정

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Range RX");
  Serial.println("Setup over........");

 

}

void loop()
{
  struct Anchor_table anchors[MAX_ANCHORS]; //앵커들 정보 저장할 구조체 선언
  init_anchor(anchors); //앵커테이블 초기화
  
  // poll 메시지 준비 및 전송 단계
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
  //tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb; //시퀀스 번호 필드에 현재 프레임번호 기록 
  
  uint8_t seq_tx = frame_seq_nb;
  tx_poll_msg[ALL_MSG_SN_IDX] = seq_tx;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // 전송 직전에 이전 전송 완료 플래그 초기화
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */ // DW3000 내부 송신 버퍼에 tx_poll_msg 적재
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);




  uint32_t start_time = millis();
 


  // poll 송신 후 수신 상태 확인
  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) //정상 수신 or 타임아웃 or 에러 발생
  {
  };
  

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  while(millis() - start_time < 16) {
    status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) //응답 메시지 수신 확인
    {
      uint32_t frame_len;

      /* Clear good RX frame event in the DW IC status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); //수신한 프레임 이벤트 초기화

      /* A frame has been received, read it into the local buffer. */
      // 프레임 길이 확인
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK; 
      if (frame_len <= sizeof(rx_buffer))
      {
        // RX버퍼에서 수신 데이터 읽기
        dwt_readrxdata(rx_buffer, frame_len, 0); //RX 버퍼에 있는 값 rx_buffer로 복사


        
        memcpy(anchor_id, rx_buffer + 5, 4); // 앵커한테 받은 resp 메시지에서 앵커 아이디 가져오기
        
        // Serial.print("현재 앵커 아이디: ");
        // Serial.println(anchor_id);

        //동적으로 테이블 아이디 관리하기
        int idx = add_table(anchor_id);
        //Serial.println(idx);
        

        for (int i = 0; i < MAX_ANCHORS; i++) {
          if (strncmp(anchor_id, anchor_table[i], 4) == 0 && received[i] == false ) {

            if (rx_buffer[0] == 0x41 && rx_buffer[1] == 0x88 && rx_buffer[9]==0xE1) { //프레임검사
              if (rx_buffer[ALL_MSG_SN_IDX] != seq_tx) { //보낸 seq번호랑 받은 seq번호랑 일치하는지 확인
                dwt_rxenable(DWT_START_RX_IMMEDIATE);                       
              } else {
                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32_t rtd_init, rtd_resp;
                float clockOffsetRatio;

                //앵커아이디 저장
                strncpy(anchors[i].anchor_id, anchor_id, sizeof(anchors[i].anchor_id) - 1);
                anchors[i].anchor_id[sizeof(anchors[i].anchor_id) - 1] = '\0'; 

                //타임스탬프 추출
                /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                poll_tx_ts = dwt_readtxtimestamplo32(); //poll을 보낸 시각
                resp_rx_ts = dwt_readrxtimestamplo32(); //resp을 받은 시각

                /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26); //클럭 오차 보정값 계산

                /* Get timestamps embedded in response message. */
                
                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts); 
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts); 

                /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                rtd_init = resp_rx_ts - poll_tx_ts; 
                rtd_resp = resp_tx_ts - poll_rx_ts; 

                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;

         

                //anchors[i].distance = round(distance*100) / 100;
                anchors[i].distance = distance;
                anchors[i].seq = seq_tx;



                //distances[i] = round(distance*100) / 100;
                distances[i] = distance;
                received[i] = true;

                
                // /* Display computed distance on LCD. */
                // Serial.print(anchor_id);
                // Serial.print(": ");
                // Serial.print(distance);
                // Serial.print("m ");
                // Serial.println();
          
                //snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
                test_run_info((unsigned char *)dist_str);
              } 
            }
          } 
        }
      }
      dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
    else
    {
      /* Clear RX error/timeout events in the DW IC status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
      // dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }
  }

  frame_seq_nb++; //프레임 시퀀스 번호 증가
  // for (int i = 0; i < MAX_ANCHORS; i++) {
  //   Serial.print(anchor_table[i]);
  //   Serial.print(": ");
  //   Serial.print(distances[i]);
  //   Serial.print("m ");
  // }


  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (received[i] == true) {
      Serial.print(anchors[i].anchor_id);
      Serial.print(": ");
      Serial.print(anchors[i].distance);
      Serial.print("m ");
    }
  }
  Serial.println();



  //mqtt에서 연결을 유지하고 수신 이벤트 처리함
  client.loop();


  //직렬화(json으로 변환)
  StaticJsonDocument<300> doc;


  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (received[i] == true) {
      JsonObject anchor = doc.createNestedObject(anchors[i].anchor_id);
      anchor["dist"] = anchors[i].distance;
      anchor["seq"] = anchors[i].seq;
    }
  }

  char msg[300];
  serializeJson(doc, msg); //JsonObect를 char형태로 변환
  
  // 토픽 발행
  client.publish(topic, msg, 2);

  for (int i = 0; i < MAX_ANCHORS; i++) {
    received[i] = false;
  }

  /* Execute a delay between ranging exchanges. */
  Sleep(RNG_DELAY_MS); //다음 루프까지 대기
}



//앵커 구조체 초기화
void init_anchor(struct Anchor_table anchors[]) {
  //const char *ids[MAX_ANCHORS] = {"ANC0", "ANC3", "ANC4", "ANC6"};
  for (int i = 0; i < MAX_ANCHORS; i++) {
    strncpy(anchors[i].anchor_id, anchor_table[i], sizeof(anchors[i].anchor_id) - 1);
    anchors[i].anchor_id[sizeof(anchors[i].anchor_id) - 1] = '\0';
    anchors[i].distance = 0.0;
    anchors[i].seq = 0;
  }
}


int add_table(const char *id) {
  //현재 아이디가 이미 배열에 존재하는 경우
  //Serial.println(id);
  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (is_used[i] == true && strncmp(anchor_table[i], id, 4) == 0) {
      return i; 
    }
  }
  // 새로 추가
  for (int i = 0; i < MAX_ANCHORS; i++) {
    if (is_used[i] == false || anchor_table[i][0] == '\0') {
      strncpy(anchor_table[i], id, 4);
      anchor_table[i][4] = '\0'; 
      is_used[i] = true;
      return i;  
    }
  }

}


