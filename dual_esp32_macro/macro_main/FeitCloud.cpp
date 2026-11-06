#include "LocalClock.h"
#include "Weather.h"
#include "HotList.h"
#include "Speech.h"
#include "ChineseFont.h"
#include "DogAnimation.h"
#include "SdDiagnostic.h"
#include "FeitCloud.h"
#include "BrightnessControl.h"
#include "GroupPower.h"
#include "ColorPreset.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <mbedtls/md.h>
#include <time.h>
#include "feit_connection_timing.h"
namespace {
#include "FeitSecrets.h"
constexpr uint8_t kBulbCount = 3;
const char* bulbIds[kBulbCount] = {TUYA_DEVICE_ID, TUYA_DEVICE_ID_2, TUYA_DEVICE_ID_3};
struct BulbState {
  bool power=false, known=false, colourMode=false;
  uint16_t brightness=1000, hue=0, saturation=1000;
};
BulbState bulbs[kBulbCount];
String g_token = "";
unsigned long g_token_expire_ms = 0;

// ---- HMAC-SHA256，返回大写 hex ----
String hmacSHA256(const String& key, const String& data) {
  unsigned char out[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_setup(&ctx, info, 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)key.c_str(), key.length());
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)data.c_str(), data.length());
  mbedtls_md_hmac_finish(&ctx, out);
  mbedtls_md_free(&ctx);
  String hex;
  hex.reserve(64);
  for (int i = 0; i < 32; i++) {
    if (out[i] < 16) hex += "0";
    hex += String(out[i], HEX);
  }
  hex.toUpperCase();
  return hex;
}

// ---- SHA256，返回小写 hex（涂鸦 canonical 签名用） ----
String sha256hex(const String& data) {
  unsigned char out[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  mbedtls_md_setup(&ctx, info, 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, (const unsigned char*)data.c_str(), data.length());
  mbedtls_md_finish(&ctx, out);
  mbedtls_md_free(&ctx);
  String hex;
  hex.reserve(64);
  for (int i = 0; i < 32; i++) {
    if (out[i] < 16) hex += "0";
    hex += String(out[i], HEX);
  }
  hex.toLowerCase();
  return hex;
}

// ---- 涂鸦 canonical 签名（经官方 SDK 验证通过） ----
// str_to_sign = METHOD + "\n" + SHA256(body) + "\n" + "\n" + PATH[?query]
// message = client_id + [access_token] + t + str_to_sign
String tuyaSign(const String& method, const String& pathQuery,
                const String& body, const String& token, const String& t) {
  String sts = method + "\n" + sha256hex(body) + "\n\n" + pathQuery;
  String msg = String(TUYA_CLIENT_ID) + token + t + sts;
  return hmacSHA256(TUYA_SECRET, msg);
}

// 13位毫秒时间戳（NTP校时后）
String nowMs2() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  char buf[24];
  snprintf(buf, sizeof(buf), "%llu", (unsigned long long)tv.tv_sec * 1000 + tv.tv_usec / 1000);
  return String(buf);
}

// Persistent HTTPS objects: http.end() clears per-request headers while retaining
// the TLS socket only when HTTPClient confirms that the server allows reuse.
// Observe the existing HTTPClient transport without changing its request parser.
class TimedCloudTLS : public WiFiClientSecure {
public:
  using WiFiClientSecure::connect;
  feit_diag::ConnectPhases connectionPhases;
  String resolvedAddress;
  int connectionError = 0;
  uint32_t connectMs = 0, writeMs = 0, writeEndAt = 0, firstByteAt = 0;
  uint32_t writeBytes = 0, writeCalls = 0, connectCalls = 0;
  bool sawFirstByte = false, measuring = false;

  void beginMeasurement() {
    connectionPhases = feit_diag::ConnectPhases{};
    resolvedAddress = "-";
    connectionError = 0;
    connectMs = writeMs = writeEndAt = firstByteAt = 0;
    writeBytes = writeCalls = connectCalls = 0;
    sawFirstByte = false;
    measuring = true;
  }
  int connect(const char* host, uint16_t port, int32_t timeout) override {
    const uint32_t at = millis();
    ++connectCalls;
    _timeout = timeout;
    IPAddress address;
    connectionPhases.dnsAttempted = true;
    uint32_t phaseAt = millis();
    connectionPhases.dnsOk = Network.hostByName(host, address) == 1;
    connectionPhases.dnsMs = millis() - phaseAt;
    if (!connectionPhases.dnsOk) {
      connectionError = -1;
      connectMs += millis() - at;
      return 0;
    }
    resolvedAddress = address.toString();
    // Preserve the hostname for SNI and certificate verification after DNS.
    int ret = feit_diag::startMeasuredSSL(sslclient.get(), address, port, host, _timeout,
                 _CA_cert, _use_ca_bundle, _cert, _private_key, _pskIdent, _psKey,
                 _use_insecure, _alpn_protos, connectionPhases);
    if (ret >= 0) {
      connectionPhases.handshakeAttempted = true;
      phaseAt = millis();
      ret = ssl_starttls_handshake(sslclient.get());
      connectionPhases.handshakeMs = millis() - phaseAt;
      if (ret >= 0 && mbedtls_ssl_get_verify_result(&sslclient->ssl_ctx) != 0) ret = MBEDTLS_ERR_X509_CERT_VERIFY_FAILED;
      connectionPhases.handshakeOk = ret >= 0;
    }
    sslclient->last_error = ret;
    connectionError = ret < 0 ? ret : 0;
    connectMs += millis() - at;
    if (ret < 0) { stop(); return 0; }
    _connected = true;
    return 1;
  }
  size_t write(const uint8_t* data, size_t size) override {
    uint32_t at = millis();
    size_t written = WiFiClientSecure::write(data, size);
    if (measuring) {
      writeEndAt = millis();
      writeMs += writeEndAt - at;
      writeBytes += written;
      ++writeCalls;
    }
    return written;
  }
  size_t write(uint8_t data) override { return write(&data, 1); }
  int available() override {
    int count = WiFiClientSecure::available();
    if (measuring && writeBytes > 0 && count > 0 && !sawFirstByte) {
      firstByteAt = millis();
      sawFirstByte = true;
    }
    return count;
  }
};
TimedCloudTLS cloudTls;
HTTPClient cloudHttp;
String cloudConnectionHost;
// Accessed only by setup/loop; button task sends timestamped values via queue.
bool buttonTraceActive = false;
uint32_t buttonTraceId = 0;
uint32_t buttonTracePressedAt = 0;

String bridgeRequest(const String& method, const String& host, const String& path,
                     const String& extraHeaders, const String& body) {
  const uint32_t started = millis();
  if (buttonTraceActive)
    Serial.printf("[TIMING] id=%lu stage=request_begin method=%s press_to_request=%lu ms\n",
                  (unsigned long)buttonTraceId, method.c_str(),
                  (unsigned long)(started - buttonTracePressedAt));
  if (cloudConnectionHost != host || WiFi.status() != WL_CONNECTED) {
    cloudTls.stop();
    cloudConnectionHost = host;
  }
  cloudTls.setCACert(TUYA_CA);
  cloudTls.setHandshakeTimeout(10);
  cloudHttp.setConnectTimeout(5000);
  cloudHttp.setTimeout(8000);
  cloudHttp.setReuse(true);

  for (int attempt = 1; attempt <= 2; ++attempt) {
    const uint32_t attemptAt = millis();
    const bool reuseCandidate = cloudTls.connected();
    if (!cloudHttp.begin(cloudTls, String("https://") + host + path)) {
      cloudTls.stop();
      return "[HTTP begin failed]";
    }
    // begin/end clear request headers; rebuild authorization for every request.
    int offset = 0;
    while (offset < (int)extraHeaders.length()) {
      int end = extraHeaders.indexOf("\r\n", offset);
      if (end < 0) end = extraHeaders.length();
      String line = extraHeaders.substring(offset, end);
      int colon = line.indexOf(':');
      if (colon > 0) {
        String value = line.substring(colon + 1); value.trim();
        cloudHttp.addHeader(line.substring(0, colon), value);
      }
      offset = end + 2;
    }
    const uint32_t sendCallAt = millis();
    if (buttonTraceActive)
      Serial.printf("[TIMING] id=%lu stage=sendRequest method=%s attempt=%d press_to_send_call=%lu ms (includes connect inside call)\n",
                    (unsigned long)buttonTraceId, method.c_str(), attempt,
                    (unsigned long)(sendCallAt - buttonTracePressedAt));
    cloudTls.beginMeasurement();
    const uint32_t transportAt = millis();
    int status = cloudHttp.sendRequest(method.c_str(), body);
    uint32_t headersAt = millis();
    cloudTls.measuring = false;
    const long waitFirstMs = cloudTls.sawFirstByte ? (long)(cloudTls.firstByteAt - cloudTls.writeEndAt) : -1;
    const long readHeadersMs = cloudTls.sawFirstByte ? (long)(headersAt - cloudTls.firstByteAt) : -1;

    String resp = status > 0 ? cloudHttp.getString() : String("[HTTP error ") + status + "]";
    uint32_t bodyAt = millis();
    if (cloudTls.connectCalls > 0) {
      const auto& phases = cloudTls.connectionPhases;
      Serial.printf("[CONNECT] ip=%s dns=%lu ms dns_ok=%d tcp=%lu ms tcp_attempted=%d tcp_ok=%d tls_setup=%lu ms setup_attempted=%d setup_ok=%d tls_handshake=%lu ms handshake_attempted=%d handshake_ok=%d total=%lu ms error=%d\n",
                    cloudTls.resolvedAddress.c_str(), (unsigned long)phases.dnsMs, phases.dnsOk,
                    (unsigned long)phases.tcpMs, phases.tcpAttempted, phases.tcpOk,
                    (unsigned long)phases.setupMs, phases.setupAttempted, phases.setupOk,
                    (unsigned long)phases.handshakeMs, phases.handshakeAttempted, phases.handshakeOk,
                    (unsigned long)cloudTls.connectMs, cloudTls.connectionError);
    }
    Serial.printf("[PHASE] method=%s attempt=%d connect_calls=%lu connect_dns_tcp_tls=%lu ms write_calls=%lu sent_bytes=%lu tls_write=%lu ms wait_first_byte=%ld ms read_headers=%ld ms transport_total=%lu ms\n",
                  method.c_str(), attempt, (unsigned long)cloudTls.connectCalls,
                  (unsigned long)cloudTls.connectMs, (unsigned long)cloudTls.writeCalls,
                  (unsigned long)cloudTls.writeBytes, (unsigned long)cloudTls.writeMs,
                  waitFirstMs, readHeadersMs, (unsigned long)(headersAt - transportAt));
    if (buttonTraceActive)
      Serial.printf("[TIMING] id=%lu stage=response method=%s attempt=%d press_to_headers=%lu ms press_to_body=%lu ms status=%d\n",
                    (unsigned long)buttonTraceId, method.c_str(), attempt,
                    (unsigned long)(headersAt - buttonTracePressedAt),
                    (unsigned long)(bodyAt - buttonTracePressedAt), status);
    // Fully consume the response before permitting the next request.
    cloudHttp.end();
    if (status < 0) cloudTls.stop();
    Serial.printf("[HTTP] %s %s attempt=%d connection=%s status=%d headers=%lu ms body=%lu ms total=%lu ms keepalive=%s\n",
                  method.c_str(), path.c_str(), attempt, reuseCandidate ? "REUSE candidate" : "NEW",
                  status, (unsigned long)(headersAt - attemptAt),
                  (unsigned long)(bodyAt - headersAt), (unsigned long)(millis() - started),
                  cloudTls.connected() ? "yes" : "no");
    // -1 means connect() failed before HTTP headers/payload were sent.
    // Do not replay a POST after an ambiguous send/response failure.
    if (status == HTTPC_ERROR_CONNECTION_REFUSED && attempt == 1 && WiFi.status() == WL_CONNECTED) {
      Serial.println("[HTTP] Connection failed; reconnect and retry once");
      delay(100);
      continue;
    }
    return resp;
  }
  return "[HTTP connection failed after retry]";
}

// ---- 取 token（有效期2小时，自动缓存） ----
bool tuyaGetToken() {
  if (g_token.length() > 0 && static_cast<int32_t>(g_token_expire_ms - millis()) > 0) return true;
  String t = nowMs2();
  String pathQ = "/v1.0/token?grant_type=1";
  String sign = tuyaSign("GET", pathQ, "", "", t);
  String headers = String("client_id: ") + TUYA_CLIENT_ID + "\r\n"
                 + "access_token: \r\n"
                 + "t: " + t + "\r\n"
                 + "sign_method: HMAC-SHA256\r\n"
                 + "sign: " + sign + "\r\n";
  String resp = bridgeRequest("GET", TUYA_HOST, pathQ, headers, "");
  JsonDocument doc;
  if (deserializeJson(doc, resp)) { Serial.println("[FEIT] Token response invalid"); return false; }
  if (!doc["success"].as<bool>()) { Serial.printf("[FEIT] Token rejected code=%d\n", doc["code"].as<int>()); return false; }
  g_token = doc["result"]["access_token"].as<String>();
  long exp = doc["result"]["expire_time"].as<long>();  // 秒，一般7200
  g_token_expire_ms = millis() + static_cast<uint32_t>(exp > 300 ? exp - 300 : (exp > 1 ? exp - 1 : 1)) * 1000UL;  // 提前5分钟过期
  Serial.println("token ok");
  return g_token.length() > 0;
}

// ---- 发控制指令，commandsJson 如 {"switch_led":true} ----
bool tuyaCommand(const String& commandsJson, uint8_t bulb=0) {
  if (!tuyaGetToken()) return false;
  String t = nowMs2();
  String path = String("/v1.0/iot-03/devices/") + bulbIds[bulb] + "/commands";
  String body = String("{\"commands\":[") + commandsJson + "]}";
  String sign = tuyaSign("POST", path, body, g_token, t);
  String headers = String("client_id: ") + TUYA_CLIENT_ID + "\r\n"
                 + "access_token: " + g_token + "\r\n"
                 + "t: " + t + "\r\n"
                 + "sign_method: HMAC-SHA256\r\n"
                 + "sign: " + sign + "\r\n"
                 + "Content-Type: application/json\r\n";
  String resp = bridgeRequest("POST", TUYA_HOST, path, headers, body);
  Serial.println("tuya -> " + resp);
  JsonDocument doc;
  if (deserializeJson(doc, resp)) return false;
  if (doc["code"].as<int>() == 1010) g_token = "";
  return doc["success"].as<bool>() && doc["result"].as<bool>();
}

// Cached power reflects a successful status query or cloud-accepted target.
// External app changes require a serial "status" to resynchronize.
// Each bulb owns its power/known cache; a failure never overwrites the other bulb.

bool setBulbPower(bool target, uint8_t bulb=0) {
  bool ok = tuyaCommand(target ? "{\"code\":\"switch_led\",\"value\":true}"
                               : "{\"code\":\"switch_led\",\"value\":false}", bulb);
  bulbs[bulb].known = ok;
  if (ok) bulbs[bulb].power = target;
  return ok;
}

// Serial and button controls share the same cached target.
bool bulbOn()  { return setBulbPower(true); }
bool bulbOff() { return setBulbPower(false); }
bool bulbBright(int v, uint8_t bulb=0){
  v=constrain(v,10,1000);
  String command;
  if (bulbs[bulb].colourMode) {
    command="{\"code\":\"colour_data_v2\",\"value\":{\"h\":"+String(bulbs[bulb].hue)
           +",\"s\":"+String(bulbs[bulb].saturation)+",\"v\":"+String(v)+"}}";
  } else command="{\"code\":\"bright_value_v2\",\"value\":"+String(v)+"}";
  const bool ok=tuyaCommand(command,bulb);
  if(ok)bulbs[bulb].brightness=v;
  return ok;
}
bool bulbTemp(int v){   // 0~1000, 0偏暖 1000偏冷
  v = constrain(v, 0, 1000);
  return tuyaCommand("{\"code\":\"temp_value_v2\",\"value\":" + String(v) + "}");
}
bool bulbColor(int h, int s, int v){ // h:0-360 s:0-1000 v:0-1000
  h = constrain(h, 0, 360); s = constrain(s, 0, 1000); v = constrain(v, 10, 1000);
  char buf[96];
  snprintf(buf, sizeof(buf),
    "{\"code\":\"colour_data_v2\",\"value\":{\"h\":%d,\"s\":%d,\"v\":%d}}", h, s, v);
  const bool ok=tuyaCommand(String(buf));
  if(ok) {
    bulbs[0].hue=h; bulbs[0].saturation=s;
    if(bulbs[0].colourMode)bulbs[0].brightness=v;
  }
  return ok;
}

bool queryBulbPower(bool& power, bool printResponse, uint8_t bulb=0) {
  bulbs[bulb].known = false;
  if (!tuyaGetToken()) return false;
  String t = nowMs2();
  String path = String("/v1.0/iot-03/devices/") + bulbIds[bulb] + "/status";
  String sign = tuyaSign("GET", path, "", g_token, t);
  String headers = String("client_id: ") + TUYA_CLIENT_ID + "\r\n"
                 + "access_token: " + g_token + "\r\n"
                 + "t: " + t + "\r\n"
                 + "sign_method: HMAC-SHA256\r\n"
                 + "sign: " + sign + "\r\n";
  String resp = bridgeRequest("GET", TUYA_HOST, path, headers, "");
  if (printResponse) Serial.println(resp);
  JsonDocument doc;
  if (deserializeJson(doc, resp) || !doc["success"].as<bool>()) return false;
  bool found=false;
  uint16_t whiteBrightness=bulbs[bulb].brightness, colorBrightness=bulbs[bulb].brightness;
  for (JsonObject item : doc["result"].as<JsonArray>()) {
    const String code=item["code"].as<String>();
    if(code=="switch_led" && item["value"].is<bool>()) {
      power=item["value"].as<bool>(); bulbs[bulb].power=power; found=true;
    } else if(code=="work_mode") bulbs[bulb].colourMode=item["value"].as<String>()=="colour";
    else if(code=="bright_value_v2") whiteBrightness=constrain(item["value"].as<int>(),10,1000);
    else if(code=="colour_data_v2") {
      JsonDocument color;
      if(item["value"].is<JsonObject>())color.set(item["value"]);
      else deserializeJson(color,item["value"].as<String>());
      if(color["h"].is<int>() && color["s"].is<int>() && color["v"].is<int>()) {
        bulbs[bulb].hue=constrain(color["h"].as<int>(),0,360);
        bulbs[bulb].saturation=constrain(color["s"].as<int>(),0,1000);
        colorBrightness=constrain(color["v"].as<int>(),10,1000);
      }
    }
  }
  bulbs[bulb].known=found;
  bulbs[bulb].brightness=bulbs[bulb].colourMode?colorBrightness:whiteBrightness;
  return found;
}


bool setBulbPreset(bool colored,uint8_t bulb,uint16_t hue,uint16_t saturation) {
  bool power;
  if(!bulbs[bulb].known && !queryBulbPower(power,false,bulb))return false;
  const uint16_t level=bulbs[bulb].brightness;
  String commands=String("{\"code\":\"work_mode\",\"value\":\"")+(colored?"colour":"white")+"\"},";
  if(colored) {
    commands+="{\"code\":\"colour_data_v2\",\"value\":{\"h\":"+String(hue)
             +",\"s\":"+String(saturation)+",\"v\":"+String(level)+"}}";
  } else {
    commands+="{\"code\":\"temp_value_v2\",\"value\":"+String(colorpreset::kWarmTemperature)
             +"},{\"code\":\"bright_value_v2\",\"value\":"+String(level)+"}";
  }
  // Preserve the cached power state explicitly; color selection must not turn an OFF lamp on.
  commands+=String(",{\"code\":\"switch_led\",\"value\":")+(bulbs[bulb].power?"true":"false")+"}";
  const bool ok=tuyaCommand(commands,bulb);
  if(ok) {
    bulbs[bulb].colourMode=colored;
    if(colored) {bulbs[bulb].hue=hue;bulbs[bulb].saturation=saturation;}
  } else bulbs[bulb].known=false;
  return ok;
}
colorpreset::Toggle pinkToggle, orangeToggle;

enum class Kind : uint8_t { MasterColor, MasterToggle, Toggle, On, Off, Status, Bright, Temp, Color };
struct Command { Kind kind; uint32_t at; int a=0,b=0,c=0; bool fromPot=false; uint8_t sourceKey=0; uint8_t bulb=0; };
QueueHandle_t commands=nullptr;
bool displayResetRequested=false; // Only accessed by the UI/serial task.
int8_t pageRequested=-1;
FeitCloud::ScreenRequest screenRequested=FeitCloud::ScreenRequest::None;
FeitCloud::SpeechRequest speechRequested=FeitCloud::SpeechRequest::None;
int8_t hotViewRequested=-1;
portMUX_TYPE stateMux=portMUX_INITIALIZER_UNLOCKED;
FeitCloud::State shared;
BrightnessControl brightness[kBulbCount];
void publish(bool busy) {
  const bool wifi = WiFi.status()==WL_CONNECTED;
  portENTER_CRITICAL(&stateMux);
  shared.wifi=wifi;
  shared.known=bulbs[0].known; shared.power=bulbs[0].power; shared.busy=busy;
  shared.secondKnown=bulbs[1].known; shared.secondPower=bulbs[1].power;
  shared.thirdKnown=bulbs[2].known; shared.thirdPower=bulbs[2].power;
  portEXIT_CRITICAL(&stateMux);
}
bool enqueue(Command command) {
  Weather::userActivity();
  if (!commands || xQueueSend(commands,&command,0)!=pdTRUE) {
    Serial.println("[FEIT] Queue full/unavailable; command NOT queued"); return false;
  }
  return true;
}
void worker(void*) {
  WiFi.mode(WIFI_STA); WiFi.setAutoReconnect(true); WiFi.begin(ssid,password);
  configTzTime(LocalClock::kTimezone,"pool.ntp.org","time.nist.gov");
  uint32_t reconnectAt=millis(), primeAt=0; bool primed[kBulbCount]={}; bool wasWifi=false;
  uint8_t nextBrightness=0;
  for (;;) {
    if(ChineseFont::uploadActive()){vTaskDelay(pdMS_TO_TICKS(20));continue;}
    const bool wifi=WiFi.status()==WL_CONNECTED;
    if (!wifi) {
      if(wasWifi) { cloudHttp.end(); cloudTls.stop(); for (uint8_t i=0;i<kBulbCount;++i) { bulbs[i].known=false; primed[i]=false; } }
      if(Speech::due())Speech::poll();
      wasWifi=false; publish(false);
      if(uint32_t(millis()-reconnectAt)>=15000) { reconnectAt=millis(); WiFi.reconnect(); Serial.println("[FEIT] Waiting for WiFi"); }
      vTaskDelay(pdMS_TO_TICKS(50)); continue;
    }
    if(!wasWifi) { Serial.println("[FEIT] WiFi connected; synchronizing clock"); wasWifi=true; }
    if(time(nullptr)<1700000000) { publish(false); vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if(primeAt==0 || uint32_t(millis()-primeAt)>=30000) {
      for (uint8_t i=0;i<kBulbCount;++i) {
        if (primed[i]) continue;
        publish(true); bool power;
        primed[i]=queryBulbPower(power,false,i);
        Serial.printf("[BULB%u] Startup state %s\n",i+1,
                      primed[i]?(bulbs[i].power?"ON":"OFF"):"unavailable; will retry");
      }
      primeAt=millis(); publish(false);
    }
    Command command;
    if(xQueueReceive(commands,&command,pdMS_TO_TICKS(20))!=pdTRUE) {
      // Power commands take priority. Each lamp retains its own newest value
      // while OFF/unknown. Round-robin prevents slow requests starving a lamp.
      bool pending=false;
      for (uint8_t offset=0;offset<kBulbCount;++offset) {
        const uint8_t i=(nextBrightness+offset)%kBulbCount;
        if (!bulbs[i].known || !bulbs[i].power) continue;
        uint16_t value=0;
        portENTER_CRITICAL(&stateMux);
        pending=brightness[i].take(millis(),value);
        portEXIT_CRITICAL(&stateMux);
        if (!pending) continue;
        command={Kind::Bright,millis(),value,0,0,true,0,i};
        nextBrightness=(i+1)%kBulbCount;
        break;
      }
      if (!pending) {
        if(Speech::due()) {
          cloudHttp.end();cloudTls.stop();cloudConnectionHost="";Speech::poll();
        } else if(Speech::playing()) {
          // Background TLS waits for audio; queued lamp actions still have priority.
        } else if(HotList::due()) {
          cloudHttp.end();cloudTls.stop();cloudConnectionHost="";HotList::poll();
        } else if(Weather::due()) {
          cloudHttp.end();cloudTls.stop();cloudConnectionHost="";Weather::poll();
        }
        continue;
      }
    }
    // Do not execute old queued toggles unexpectedly after a long outage.
    if(uint32_t(millis()-command.at)>30000) { Serial.println("[FEIT] Command expired after 30s; press again"); continue; }
    if(Speech::playing() || (Speech::snapshot().mode>=Speech::Queued && Speech::snapshot().mode<=Speech::Playing)){Speech::stop();Speech::waitStopped();}
    publish(true); buttonTraceActive=true; buttonTraceId++; buttonTracePressedAt=command.at;
    bool ok=false, power;
    switch(command.kind) {
      case Kind::MasterColor: {
        const bool orange=command.sourceKey==6;
        auto& toggle=orange?orangeToggle:pinkToggle;
        const bool colored=toggle.colored();
        toggle.advance(); // Advance on every event, independently of device ACKs.
        const char* colorName=orange?"ORANGE":"PINK";
        const auto preset=colorpreset::forKey(command.sourceKey);
        Serial.printf("[COLOR] K%u -> ALL %s\n",command.sourceKey,colored?colorName:"WARM WHITE");
        uint8_t failedMask=0;
        ok=true;
        for(uint8_t i=0;i<kBulbCount;++i) {
          const bool accepted=setBulbPreset(colored,i,preset.hue,preset.saturation);
          Serial.printf("[COLOR] bulb=%u target=%s accepted=%s\n",i+1,colored?colorName:"WARM WHITE",accepted?"yes":"no");
          if(!accepted) {ok=false;failedMask|=1u<<i;}
          publish(true);
        }
        if(!ok)Serial.printf("[COLOR] Partial failure mask=0x%02x; next K%u switches to %s\n",failedMask,command.sourceKey,toggle.colored()?colorName:"WARM WHITE");
        break;
      }
      case Kind::MasterToggle: {
        uint8_t known=0, on=0;
        for (uint8_t i=0;i<kBulbCount;++i) {
          if (!bulbs[i].known) queryBulbPower(power,false,i);
          if (bulbs[i].known) known |= 1u<<i;
          if (bulbs[i].power) on |= 1u<<i;
        }
        const auto target=groupPowerTarget(known,on,(1u<<kBulbCount)-1);
        if (!target.valid) { Serial.println("[MASTER] State unknown; no group command sent"); break; }
        Serial.printf("[MASTER] K4 -> ALL %s\n",target.on?"ON":"OFF");
        ok=true;
        // Always attempt every target; partial failure is reported, never hidden.
        for (uint8_t i=0;i<kBulbCount;++i) {
          const bool accepted=setBulbPower(target.on,i);
          Serial.printf("[MASTER] bulb=%u target=%s accepted=%s\n",i+1,target.on?"ON":"OFF",accepted?"yes":"no");
          if (!accepted) ok=false;
          publish(true);
        }
        if (!ok) Serial.println("[MASTER] Partial/total failure; inspect each bulb result");
        break;
      }
      case Kind::Toggle:
        if(bulbs[command.bulb].known || queryBulbPower(power,false,command.bulb)) {
          if (command.sourceKey)
            Serial.printf("[BULB%u] K%u -> target=%s\n",command.bulb+1,command.sourceKey,bulbs[command.bulb].power?"OFF":"ON");
          else
            Serial.printf("[BULB%u] TOUCH -> target=%s\n",command.bulb+1,bulbs[command.bulb].power?"OFF":"ON");
          ok=setBulbPower(!bulbs[command.bulb].power,command.bulb);
        } else Serial.println("[FEIT] State unknown; toggle cancelled");
        break;
      case Kind::On: ok=setBulbPower(true,command.bulb); break;
      case Kind::Off: ok=setBulbPower(false,command.bulb); break;
      case Kind::Status: ok=queryBulbPower(power,true,command.bulb); break;
      case Kind::Bright:
        if (!command.fromPot) {
          portENTER_CRITICAL(&stateMux); brightness[command.bulb].cancel(); portEXIT_CRITICAL(&stateMux);
        }
        Serial.printf("[BRIGHT] bulb=%u source=%s target=%d/1000\n", command.bulb+1, command.fromPot ? "POT" : "SERIAL", command.a);
        ok=bulbBright(command.a,command.bulb);
        Serial.printf("[BRIGHT] bulb=%u accepted=%s\n",command.bulb+1,ok?"yes":"no");
        break;
      case Kind::Temp: ok=bulbTemp(command.a); break;
      case Kind::Color: ok=bulbColor(command.a,command.b,command.c); break;
    }
    Serial.printf("[FEIT] result=%s queued_to_result=%lu ms (cloud acceptance, not physical confirmation)\n",ok?"OK":"FAIL",(unsigned long)(millis()-command.at));
    buttonTraceActive=false;
    portENTER_CRITICAL(&stateMux); ++shared.completed; if(!ok)++shared.failures; portEXIT_CRITICAL(&stateMux);
    publish(false);
  }
}
} // private namespace
namespace FeitCloud {
bool begin() {
  if(commands) return true;
  commands=xQueueCreate(8,sizeof(Command));
  if(!commands) return false;
  if(xTaskCreatePinnedToCore(worker,"feit-cloud",16384,nullptr,1,nullptr,0)!=pdPASS) {
    vQueueDelete(commands); commands=nullptr; return false;
  }
  Serial.println("[FEIT] K4 all / K9 bulb1 / K8 bulb2 / K2 bulb3; serial: on/off/status/bright (suffix 2 or 3 selects bulb); temp/color/lcd"); return true;
}
void setBrightnessPercent(uint8_t percent) {
  Weather::userActivity();
  const uint32_t now=millis();
  portENTER_CRITICAL(&stateMux);
  for (uint8_t i=0;i<kBulbCount;++i) brightness[i].update(percent,now);
  portEXIT_CRITICAL(&stateMux);
}
bool toggleFromTouch(uint8_t bulb) {
  if (bulb >= kBulbCount) return false;
  return enqueue({Kind::Toggle,millis(),0,0,0,false,0,bulb});
}
bool toggle() { return enqueue({Kind::Toggle,millis(),0,0,0,false,9,0}); }
bool toggleSecond() { return enqueue({Kind::Toggle,millis(),0,0,0,false,8,1}); }
bool toggleThird() { return enqueue({Kind::Toggle,millis(),0,0,0,false,2,2}); }
bool toggleAllOrangeColor() { return enqueue({Kind::MasterColor,millis(),0,0,0,false,6}); }
bool toggleAllColor() { return enqueue({Kind::MasterColor,millis(),0,0,0,false,1}); }
bool toggleAll() { return enqueue({Kind::MasterToggle,millis(),0,0,0,false,4}); }
State state() { portENTER_CRITICAL(&stateMux); State result=shared; portEXIT_CRITICAL(&stateMux); return result; }
bool takeDisplayResetRequest() { bool value=displayResetRequested; displayResetRequested=false; return value; }
int8_t takePageRequest() { const int8_t value=pageRequested;pageRequested=-1;return value; }
ScreenRequest takeScreenRequest() {const auto result=screenRequested;screenRequested=ScreenRequest::None;return result;}
SpeechRequest takeSpeechRequest(){const auto result=speechRequested;speechRequested=SpeechRequest::None;return result;}
int8_t takeHotViewRequest(){const int8_t result=hotViewRequested;hotViewRequested=-1;return result;}
void serviceSerial() {
  if(ChineseFont::receiving() || doganim::receiving())return;
  static char line[96]; static size_t length=0; static bool overflow=false;
  for(unsigned n=0;n<32 && Serial.available();++n) {
    char ch=Serial.read(); if(ch=='\r')continue;
    if(ch!='\n') { if(length+1<sizeof(line))line[length++]=ch; else overflow=true; continue; }
    line[length]=0;
    int requestedPage=-1;char pageTail=0;
    if(!overflow && sscanf(line,"page %d %c",&requestedPage,&pageTail)==1 && requestedPage>=0 && requestedPage<=3) {
      pageRequested=requestedPage;length=0;continue;
    }
    if (!overflow && strcmp(line,"speech")==0) {Speech::status();length=0;continue;}
    if (!overflow && strcmp(line,"speech stop")==0) {Speech::stop();length=0;continue;}
    if (!overflow && strcmp(line,"speak")==0) {speechRequested=SpeechRequest::Next;length=0;continue;}
    if (!overflow && strcmp(line,"speaker test")==0) {speechRequested=SpeechRequest::Tone;length=0;continue;}
    if (!overflow && strcmp(line,"hotview hot")==0) {hotViewRequested=0;length=0;continue;}
    if (!overflow && strcmp(line,"hotview tv")==0) {hotViewRequested=1;length=0;continue;}
    if (!overflow && strcmp(line,"screen")==0) {screenRequested=ScreenRequest::Status;length=0;continue;}
    if (!overflow && strcmp(line,"screen off")==0) {screenRequested=ScreenRequest::Off;length=0;continue;}
    if (!overflow && strcmp(line,"screen on")==0) {screenRequested=ScreenRequest::On;length=0;continue;}
    if (!overflow && strcmp(line,"sd")==0) { doganim::suspend(); runSdDiagnostic(); doganim::begin();ChineseFont::begin(); length=0; continue; }
    if (!overflow && strcmp(line,"clock")==0) { LocalClock::status(); length=0; continue; }
    if (!overflow && strcmp(line,"clocktest")==0) { Serial.printf("[CLOCK] DST_TEST %s\n",LocalClock::selfTest()?"PASS":"FAIL"); length=0; continue; }
    if (!overflow && strcmp(line,"presets")==0) {
      const auto pink=colorpreset::forKey(1),orange=colorpreset::forKey(6);
      Serial.printf("[COLOR] K1=PINK h=%u s=%u K6=ORANGE h=%u s=%u; brightness/power preserved; independent warm toggles\n",pink.hue,pink.saturation,orange.hue,orange.saturation);
      length=0;continue;
    }
    if (!overflow && strcmp(line,"weather")==0) { Weather::status(); length=0; continue; }
    if (!overflow && strcmp(line,"weatherrefresh")==0) { Weather::refresh(); length=0; continue; }
    if (!overflow && strcmp(line,"hot")==0) {HotList::status();length=0;continue;}
    if (!overflow && strcmp(line,"hotrefresh")==0) {HotList::refresh();length=0;continue;}
    if (!overflow && strcmp(line,"hottest")==0) {Serial.printf("[HOT] SCHEDULE_TEST %s\n",HotList::selfTest()?"PASS":"FAIL");length=0;continue;}
    if (!overflow && strcmp(line,"font")==0) {ChineseFont::status();length=0;continue;}
    unsigned long fontBytes=0,fontCrc=0;char fontTail=0;
    if(!overflow && sscanf(line,"fontput %lu %lx %c",&fontBytes,&fontCrc,&fontTail)==2) {
      ChineseFont::startUpload(fontBytes,fontCrc);length=0;return;
    }
    if (!overflow && strcmp(line,"dogstatus")==0) { doganim::status(); length=0; continue; }
    if (!overflow && strcmp(line,"dogverify")==0) { doganim::verify(); length=0; continue; }
    unsigned long dogBytes=0,dogCrc=0; char dogTail=0;
    if(!overflow && sscanf(line,"dogput %lu %lx %c",&dogBytes,&dogCrc,&dogTail)==2) {
      doganim::startUpload(dogBytes,dogCrc);length=0;return;
    }
    if (!overflow && strcmp(line,"lcd")==0) {
      displayResetRequested=true; length=0; continue;
    }
    Command command{Kind::Status,millis()}; bool valid=!overflow && length;
    if(valid) {
      String text(line); text.trim(); int a,b,c; char tail;
      if(text=="on3") {command.kind=Kind::On;command.bulb=2;}
      else if(text=="off3") {command.kind=Kind::Off;command.bulb=2;}
      else if(text=="status3") {command.kind=Kind::Status;command.bulb=2;}
      else if(text=="on2") {command.kind=Kind::On;command.bulb=1;}
      else if(text=="off2") {command.kind=Kind::Off;command.bulb=1;}
      else if(text=="status2") {command.kind=Kind::Status;command.bulb=1;}
      else if(text=="on")command.kind=Kind::On;
      else if(text=="off")command.kind=Kind::Off;
      else if(text=="status")command.kind=Kind::Status;
      else if(sscanf(text.c_str(),"bright3 %d %c",&a,&tail)==1 && a>=10 && a<=1000) {command.kind=Kind::Bright;command.a=a;command.bulb=2;}
      else if(sscanf(text.c_str(),"bright2 %d %c",&a,&tail)==1 && a>=10 && a<=1000) {command.kind=Kind::Bright;command.a=a;command.bulb=1;}
      else if(sscanf(text.c_str(),"bright %d %c",&a,&tail)==1 && a>=10 && a<=1000) {command.kind=Kind::Bright;command.a=a;}
      else if(sscanf(text.c_str(),"temp %d %c",&a,&tail)==1 && a>=0 && a<=1000) {command.kind=Kind::Temp;command.a=a;}
      else if(sscanf(text.c_str(),"color %d %d %d %c",&a,&b,&c,&tail)==3 && a>=0 && a<=360 && b>=0 && b<=1000 && c>=10 && c<=1000) {command.kind=Kind::Color;command.a=a;command.b=b;command.c=c;}
      else valid=false;
    }
    if(valid)enqueue(command); else if(length)Serial.println("[FEIT] Invalid command");
    length=0;overflow=false;
  }
}
}
