#include "cal_store.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "cal";
static const char *NS  = "bb_cal";

void cal_store_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erased (version mismatch or partition full)");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
}

/* ── helpers ──────────────────────────────────────────────────────────────── */

static bool ns_open(nvs_open_mode_t mode, nvs_handle_t *h)
{
    return nvs_open(NS, mode, h) == ESP_OK;
}

/* ── PID ──────────────────────────────────────────────────────────────────── */

bool cal_store_load_pid(cal_pid_t *out)
{
    nvs_handle_t h;
    if (!ns_open(NVS_READONLY, &h)) return false;
    size_t sz = sizeof(*out);
    bool ok = (nvs_get_blob(h, "pid", out, &sz) == ESP_OK && sz == sizeof(*out));
    nvs_close(h);
    if (ok)
        ESP_LOGI(TAG, "PID loaded  kp=%.4g  ki=%.4g  kd=%.4g",
                 (double)out->kp, (double)out->ki, (double)out->kd);
    return ok;
}

void cal_store_save_pid(const cal_pid_t *in)
{
    nvs_handle_t h;
    if (!ns_open(NVS_READWRITE, &h)) { ESP_LOGE(TAG, "NVS open failed"); return; }
    ESP_ERROR_CHECK(nvs_set_blob(h, "pid", in, sizeof(*in)));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    ESP_LOGI(TAG, "PID saved  kp=%.4g  ki=%.4g  kd=%.4g",
             (double)in->kp, (double)in->ki, (double)in->kd);
}

/* ── Touch calibration ────────────────────────────────────────────────────── */

bool cal_store_load_touch(cal_touch_t *out)
{
    nvs_handle_t h;
    if (!ns_open(NVS_READONLY, &h)) return false;
    size_t sz = sizeof(*out);
    bool ok = (nvs_get_blob(h, "touch", out, &sz) == ESP_OK && sz == sizeof(*out));
    nvs_close(h);
    if (ok)
        ESP_LOGI(TAG, "Touch cal loaded  X[%ld-%ld]  Y[%ld-%ld]  flip=%d/%d  swap=%d",
                 (long)out->x_raw_min, (long)out->x_raw_max,
                 (long)out->y_raw_min, (long)out->y_raw_max,
                 out->flip_x, out->flip_y, out->swap_xy);
    return ok;
}

void cal_store_save_touch(const cal_touch_t *in)
{
    nvs_handle_t h;
    if (!ns_open(NVS_READWRITE, &h)) { ESP_LOGE(TAG, "NVS open failed"); return; }
    ESP_ERROR_CHECK(nvs_set_blob(h, "touch", in, sizeof(*in)));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);
    ESP_LOGI(TAG, "Touch cal saved  X[%ld-%ld]  Y[%ld-%ld]  flip=%d/%d  swap=%d",
             (long)in->x_raw_min, (long)in->x_raw_max,
             (long)in->y_raw_min, (long)in->y_raw_max,
             in->flip_x, in->flip_y, in->swap_xy);
}
