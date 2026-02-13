#include "TailscaleTask.h"
#include "PipelineConfig.h"
#include "PipelineContext.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_heap_caps.h>
#include <lwip/inet.h>
#include <time.h>

extern "C" {
#include "microlink.h"
}

namespace {

EventGroupHandle_t s_vpnEventGroup = nullptr;
microlink_t *s_ml = nullptr;
char s_vpnIpStr[16] = "";
uint8_t s_peerCount = 0;

void onConnected() {
  Serial.println("Tailscale: VPN CONNECTED (waiting for WG handshake)");
  if (s_ml) {
    uint32_t ip = microlink_get_vpn_ip(s_ml);
    microlink_vpn_ip_to_str(ip, s_vpnIpStr);
  }
  // Don't set VPN_CONNECTED_BIT here - wait for WG handshake in main loop
}

void onDisconnected() {
  Serial.println("Tailscale: VPN DISCONNECTED");
  // VPN_CONNECTED_BIT is managed by the main loop based on peer status
}

void onStateChange(microlink_state_t old_state, microlink_state_t new_state) {
  Serial.printf("Tailscale: %s -> %s\n",
                microlink_state_to_str(old_state),
                microlink_state_to_str(new_state));
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
  esp_log_level_set("ml_derp", ESP_LOG_WARN);
  esp_log_level_set("ml_conn", ESP_LOG_WARN);
  esp_log_level_set("ml_disco", ESP_LOG_INFO);   // DEBUG: NAT traversal investigation
  esp_log_level_set("ml_coord", ESP_LOG_WARN);
  esp_log_level_set("ml_stun", ESP_LOG_INFO);    // DEBUG: NAT traversal investigation

#ifdef TAILSCALE_AUTH_KEY
  Serial.printf("TS heap: internal=%lu free, total=%lu free\n",
                (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                (unsigned long)esp_get_free_heap_size());

  microlink_config_t config;
  microlink_get_default_config(&config);

  config.auth_key = TAILSCALE_AUTH_KEY;
  config.device_name = "m5stack-tab5";
  config.enable_derp = true;
  config.enable_stun = true;    // STUN enabled (DNS fallback + correct DERP server)
  config.enable_disco = true;   // DISCO enabled (peer filter prevents non-target WG crash)
  config.max_peers = 4;
  config.heartbeat_interval_ms = 25000;
  config.target_hostname = "rdk-x5";  // Only add this peer to WG

  config.on_connected = onConnected;
  config.on_disconnected = onDisconnected;
  config.on_state_change = onStateChange;

  s_ml = microlink_init(&config);
  Serial.printf("TS heap after init: internal=%lu free\n",
                (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  if (!s_ml) {
    Serial.println("TailscaleTask: microlink_init FAILED");
    vTaskDelete(nullptr);
    return;
  }

  esp_err_t ret = microlink_connect(s_ml);
  Serial.printf("TS heap after connect: internal=%lu free\n",
                (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  if (ret != ESP_OK) {
    Serial.printf("TailscaleTask: microlink_connect failed: %d\n", ret);
    microlink_deinit(s_ml);
    s_ml = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  bool vpnBitSet = false;
  uint32_t lastDiagMs = 0;

  while (1) {
    microlink_update(s_ml);

    // Update peer count for overlay display
    const microlink_peer_t *peers = nullptr;
    uint8_t count = 0;
    if (microlink_get_peers(s_ml, &peers, &count) == ESP_OK) {
      s_peerCount = count;
    }

    // Diagnostic log every 5 seconds
    uint32_t now = millis();
    if (now - lastDiagMs >= 30000) {
      microlink_state_t state = microlink_get_state(s_ml);
      bool anyUp = microlink_any_peer_up(s_ml);
      bool connected = microlink_is_connected(s_ml);
      UBaseType_t stackHWM = uxTaskGetStackHighWaterMark(nullptr);
      Serial.printf("TS diag: state=%s peers=%d anyUp=%d conn=%d vpnBit=%d stackHWM=%lu intHeap=%lu\n",
                    microlink_state_to_str(state), s_peerCount, anyUp, connected,
                    vpnBitSet, (unsigned long)stackHWM,
                    (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

      // Per-peer NAT traversal diagnostics (target peer only to avoid serial corruption)
      for (uint8_t i = 0; i < count; i++) {
        const auto &p = peers[i];
        // Only show full details for target peer
        if (strncmp(p.hostname, "rdk-x5", 6) != 0) continue;
        Serial.printf("  target[%d] %s: eps=%d derp=%d lat=%lums bestEp=%d lastSeen=%lums ago\n",
                      i, p.hostname, p.endpoint_count, p.using_derp,
                      (unsigned long)p.latency_ms, p.best_endpoint_idx,
                      p.last_seen_ms ? (unsigned long)(now - p.last_seen_ms) : 0UL);
        for (uint8_t ep = 0; ep < p.endpoint_count; ep++) {
          uint32_t hip = ntohl(p.endpoints[ep].addr.ip4);
          Serial.printf("    ep[%d] %lu.%lu.%lu.%lu:%u%s\n",
                        ep,
                        (unsigned long)((hip >> 24) & 0xFF),
                        (unsigned long)((hip >> 16) & 0xFF),
                        (unsigned long)((hip >> 8) & 0xFF),
                        (unsigned long)(hip & 0xFF),
                        p.endpoints[ep].port,
                        p.endpoints[ep].is_derp ? " (DERP)" : "");
        }
      }

      // STUN / NAT diagnostics
      microlink_stun_info_t stun;
      if (microlink_get_stun_info(s_ml, &stun) == ESP_OK && stun.public_ip != 0) {
        static const char *nat_names[] = {"unknown", "none", "cone", "symmetric"};
        const char *nat_str = (stun.nat_type < 4) ? nat_names[stun.nat_type] : "?";
        Serial.printf("  STUN: %lu.%lu.%lu.%lu:%u alt:%u delta=%d NAT=%s\n",
                      (unsigned long)((stun.public_ip >> 24) & 0xFF),
                      (unsigned long)((stun.public_ip >> 16) & 0xFF),
                      (unsigned long)((stun.public_ip >> 8) & 0xFF),
                      (unsigned long)(stun.public_ip & 0xFF),
                      stun.public_port, stun.public_port_alt,
                      stun.port_delta, nat_str);
      }
      microlink_stats_t stats;
      if (microlink_get_stats(s_ml, &stats) == ESP_OK) {
        Serial.printf("  stats: derp_relayed=%lu direct=%lu\n",
                      (unsigned long)stats.derp_packets_relayed,
                      (unsigned long)stats.direct_packets_sent);
      }

      lastDiagMs = now;
    }

    // Signal VPN ready only after at least one WG handshake completes
    bool peerUp = microlink_any_peer_up(s_ml);
    if (!vpnBitSet && peerUp) {
      Serial.println("TailscaleTask: WG tunnel active, signaling VPN ready");
      if (s_vpnEventGroup) {
        xEventGroupSetBits(s_vpnEventGroup, VPN_CONNECTED_BIT);
      }
      vpnBitSet = true;
    } else if (vpnBitSet && !peerUp) {
      Serial.println("TailscaleTask: WG tunnel lost");
      if (s_vpnEventGroup) {
        xEventGroupClearBits(s_vpnEventGroup, VPN_CONNECTED_BIT);
      }
      vpnBitSet = false;
    }

    // Faster polling when VPN active (DERP streaming needs low latency)
    vTaskDelay(pdMS_TO_TICKS(vpnBitSet ? 2 : 10));
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
