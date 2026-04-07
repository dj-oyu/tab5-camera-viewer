#include "TailscaleTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_heap_caps.h>
#include <time.h>

extern "C" {
#include "microlink.h"
}

namespace {

EventGroupHandle_t s_vpnEventGroup = nullptr;
microlink_t *s_ml = nullptr;
char s_vpnIpStr[16] = "";
uint8_t s_peerCount = 0;

const char *mlStateToStr(microlink_state_t state) {
  switch (state) {
    case ML_STATE_IDLE:         return "IDLE";
    case ML_STATE_WIFI_WAIT:    return "WIFI_WAIT";
    case ML_STATE_CONNECTING:   return "CONNECTING";
    case ML_STATE_REGISTERING:  return "REGISTERING";
    case ML_STATE_CONNECTED:    return "CONNECTED";
    case ML_STATE_RECONNECTING: return "RECONNECTING";
    case ML_STATE_ERROR:        return "ERROR";
    default:                    return "UNKNOWN";
  }
}

void onStateChange(microlink_t *ml, microlink_state_t new_state, void *) {
  Serial.printf("Tailscale: -> %s\n", mlStateToStr(new_state));
  if (new_state == ML_STATE_CONNECTED && ml) {
    uint32_t ip = microlink_get_vpn_ip(ml);
    microlink_ip_to_str(ip, s_vpnIpStr);
    Serial.printf("Tailscale: VPN IP = %s\n", s_vpnIpStr);
  }
}

void tailscaleTask(void *pvParameters) {
  // Wait for WiFi (initialized by FetchTask)
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  Serial.println("TailscaleTask: WiFi ready, syncing NTP...");

  // NTP sync is CRITICAL: WireGuard TAI64N timestamps must use real wall-clock
  // time. Without NTP, gettimeofday() returns epoch (1970), and after a reboot
  // the timestamp would be older than what the peer last saw, causing the peer
  // to reject all handshakes as replay attacks.
  esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_setservername(1, "time.google.com");
  esp_sntp_init();

  // Wait for NTP sync (max 10 seconds)
  struct tm timeinfo = {};
  for (int retry = 0; retry < 20; retry++) {
    time_t now = time(nullptr);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year >= (2026 - 1900)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  if (timeinfo.tm_year >= (2026 - 1900)) {
    time_t now = time(nullptr);
    Serial.printf("TailscaleTask: NTP synced, time=%ld (%d-%02d-%02d)\n",
                  (long)now, timeinfo.tm_year + 1900,
                  timeinfo.tm_mon + 1, timeinfo.tm_mday);
  } else {
    Serial.println("TailscaleTask: WARNING - NTP sync failed, WG handshake may be rejected");
  }

  // Suppress MicroLink logs - only show errors and state transitions
  esp_log_level_set("microlink", ESP_LOG_WARN);
  esp_log_level_set("ml_wg", ESP_LOG_WARN);
  esp_log_level_set("ml_wg_mgr", ESP_LOG_WARN);
  esp_log_level_set("ml_net_io", ESP_LOG_WARN);
  esp_log_level_set("ml_derp", ESP_LOG_WARN);
  esp_log_level_set("ml_conn", ESP_LOG_WARN);
  esp_log_level_set("ml_disco", ESP_LOG_WARN);
  esp_log_level_set("ml_coord", ESP_LOG_WARN);
  esp_log_level_set("ml_stun", ESP_LOG_WARN);

#ifdef TAILSCALE_AUTH_KEY
  Serial.printf("TS heap: internal=%lu free, total=%lu free\n",
                (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned long)esp_get_free_heap_size());

  microlink_config_t config = {};
  config.auth_key = TAILSCALE_AUTH_KEY;
  config.device_name = "m5stack-tab5";
  config.enable_derp = true;
  config.enable_stun = true;
  config.enable_disco = true;
  config.max_peers = 4;
#ifdef TAILSCALE_PRIORITY_PEER_IP
  config.priority_peer_ip = microlink_parse_ip(TAILSCALE_PRIORITY_PEER_IP);
#endif

  s_ml = microlink_init(&config);
  Serial.printf("TS heap after init: internal=%lu free\n",
                (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  if (!s_ml) {
    Serial.println("TailscaleTask: microlink_init FAILED");
    vTaskDelete(nullptr);
    return;
  }

  microlink_set_state_callback(s_ml, onStateChange, nullptr);

  esp_err_t ret = microlink_start(s_ml);
  Serial.printf("TS heap after start: internal=%lu free\n",
                (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  if (ret != ESP_OK) {
    Serial.printf("TailscaleTask: microlink_start failed: %d\n", ret);
    microlink_destroy(s_ml);
    s_ml = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  bool vpnBitSet = false;
  uint32_t lastDiagMs = 0;

  while (1) {
    // v2.x is fully async - no microlink_update() needed

    // Update peer count for overlay display
    int count = microlink_get_peer_count(s_ml);
    s_peerCount = (uint8_t)count;

    // Diagnostic log every 30 seconds
    uint32_t now = millis();
    if (now - lastDiagMs >= 30000) {
      microlink_state_t state = microlink_get_state(s_ml);
      bool connected = microlink_is_connected(s_ml);
      UBaseType_t stackHWM = uxTaskGetStackHighWaterMark(nullptr);
      Serial.printf("TS diag: state=%s peers=%d conn=%d vpnBit=%d stackHWM=%lu intHeap=%lu\n",
                    mlStateToStr(state), count, connected,
                    vpnBitSet, (unsigned long)stackHWM,
                    (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

      // Per-peer diagnostics
      microlink_peer_info_t peer;
      for (int i = 0; i < count; i++) {
        if (microlink_get_peer_info(s_ml, i, &peer) == ESP_OK) {
          Serial.printf("  peer[%d] %s: online=%d direct=%d\n",
                        i, peer.hostname, peer.online, peer.direct_path);
        }
      }

      lastDiagMs = now;
    }

    // Signal VPN ready when any peer has completed WG handshake
    bool anyOnline = false;
    {
      microlink_peer_info_t peer;
      for (int i = 0; i < count; i++) {
        if (microlink_get_peer_info(s_ml, i, &peer) == ESP_OK && peer.online) {
          anyOnline = true;
          break;
        }
      }
    }
    if (!vpnBitSet && anyOnline) {
      Serial.println("TailscaleTask: WG tunnel active, signaling VPN ready");
      if (s_vpnEventGroup) {
        xEventGroupSetBits(s_vpnEventGroup, VPN_CONNECTED_BIT);
      }
      vpnBitSet = true;
    } else if (vpnBitSet && !anyOnline) {
      Serial.println("TailscaleTask: WG tunnel lost");
      if (s_vpnEventGroup) {
        xEventGroupClearBits(s_vpnEventGroup, VPN_CONNECTED_BIT);
      }
      vpnBitSet = false;
    }

    vTaskDelay(pdMS_TO_TICKS(vpnBitSet ? 10 : 50));
  }
#else
  Serial.println("TailscaleTask: TAILSCALE_AUTH_KEY not defined, task idle");
  vTaskDelete(nullptr);
#endif
}

} // namespace

namespace TailscaleTask {

void start(PipelineContext &ctx, UBaseType_t priority, BaseType_t core) {
  if (!s_vpnEventGroup) {
    s_vpnEventGroup = xEventGroupCreate();
  }
  xTaskCreatePinnedToCore(tailscaleTask, "Tailscale", STACK_DEPTH_TAILSCALE,
                          &ctx, priority, nullptr, core);
}

EventGroupHandle_t eventGroup() { return s_vpnEventGroup; }

bool isConnected() {
  if (!s_vpnEventGroup)
    return false;
  return (xEventGroupGetBits(s_vpnEventGroup) & VPN_CONNECTED_BIT) != 0;
}

const char *vpnIpStr() { return s_vpnIpStr; }

uint8_t peerCount() { return s_peerCount; }

} // namespace TailscaleTask
