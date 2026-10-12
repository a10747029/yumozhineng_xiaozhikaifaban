#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "environment_sensors.h"
#include "esp_video.h"
#include "esp_video_init.h"
#include <cmath>
#include <algorithm>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <esp_timer.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"

#include <cstring>
#include <vector>

static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardLCD"
// Stereo microphone input for two INMP441 microphones sharing one I2S SD/DIN
// wire. Connect one microphone L/R pin to GND and the other to 3.3V.
class StereoMicNoAudioCodec final : public NoAudioCodec {
private:
    static constexpr size_t kTdoaWindowFrames = 2048;
    static constexpr int kMaxLagSamples = 8;
    static constexpr uint32_t kTdoaUpdateMs = 1000;
    static constexpr int64_t kSilenceTimeoutUs = 3000000;

    std::array<int16_t, kTdoaWindowFrames> left_ring_{};
    std::array<int16_t, kTdoaWindowFrames> right_ring_{};
    std::array<double, kMaxLagSamples * 2 + 1> correlations_{};

    size_t ring_write_index_ = 0;
    size_t ring_frames_ = 0;
    int64_t last_tdoa_update_us_ = 0;
    int64_t last_voice_us_ = 0;
    static constexpr double kMinCorrelation = 0.20;
    static constexpr double kMicSpacingM = 0.05;
    static constexpr double kSpeedOfSoundMps = 343.0;
    static constexpr double kCenterAngleDeg = 20.0;
    static constexpr double kDirectionRmsThreshold = 900.0;
    std::string displayed_direction_;

public:
    StereoMicNoAudioCodec(int input_sample_rate, int output_sample_rate,
                          gpio_num_t spk_bclk, gpio_num_t spk_ws,
                          gpio_num_t spk_dout, gpio_num_t mic_bclk,
                          gpio_num_t mic_ws, gpio_num_t mic_din) {
        duplex_ = false;
        input_sample_rate_ = input_sample_rate;
        output_sample_rate_ = output_sample_rate;
	// Xiaozhi's voice/AFE path receives mono. Keep both raw I2S slots
        // inside Read() for level/TDOA analysis, but feed the left mic onward.
        input_channels_ = 1;
        output_channels_ = 1;

        i2s_chan_config_t chan_cfg = {
            .id = XIAOZHI_I2S_PORT(0),
            .role = I2S_ROLE_MASTER,
            .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
            .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
            .auto_clear_after_cb = true,
            .auto_clear_before_cb = false,
            .intr_priority = 0,
        };
        ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, nullptr));

        i2s_std_config_t tx_cfg = {
            .clk_cfg = {
                .sample_rate_hz = static_cast<uint32_t>(output_sample_rate_),
                .clk_src = I2S_CLK_SRC_DEFAULT,
#if SOC_I2S_HW_VERSION_2
                .ext_clk_freq_hz = 0,
#endif
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            },
            .slot_cfg = {
                .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode = I2S_SLOT_MODE_MONO,
                .slot_mask = I2S_STD_SLOT_LEFT,
                .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
                .ws_pol = false,
                .bit_shift = true,
#if SOC_I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false,
#endif
            },
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = spk_bclk,
                .ws = spk_ws,
                .dout = spk_dout,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &tx_cfg));

        chan_cfg.id = XIAOZHI_I2S_PORT(1);
        ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle_));

        i2s_std_config_t rx_cfg = {
            .clk_cfg = {
                .sample_rate_hz = static_cast<uint32_t>(input_sample_rate_),
                .clk_src = I2S_CLK_SRC_DEFAULT,
#if SOC_I2S_HW_VERSION_2
                .ext_clk_freq_hz = 0,
#endif
                .mclk_multiple = I2S_MCLK_MULTIPLE_256,
            },
            .slot_cfg = {
                .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                .slot_mode = I2S_SLOT_MODE_STEREO,
                .slot_mask = I2S_STD_SLOT_BOTH,
                .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
                .ws_pol = false,
                .bit_shift = true,
#if SOC_I2S_HW_VERSION_2
                .left_align = true,
                .big_endian = false,
                .bit_order_lsb = false,
#endif
            },
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = mic_bclk,
                .ws = mic_ws,
                .dout = I2S_GPIO_UNUSED,
                .din = mic_din,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
            },
        };
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &rx_cfg));
        ESP_LOGI(TAG, "Stereo microphone input initialized at %d Hz", input_sample_rate_);
    }

protected:
    int Read(int16_t* dest, int samples) override {
    if (dest == nullptr || samples <= 0) {
        return 0;
    }

    // Caller requests mono samples. Read an interleaved stereo I2S window.
    std::vector<int32_t> raw(static_cast<size_t>(samples) * 2);
    size_t bytes_read = 0;

    const esp_err_t err = i2s_channel_read(
        rx_handle_,
        raw.data(),
        raw.size() * sizeof(int32_t),
        &bytes_read,
        pdMS_TO_TICKS(200));

    if (err != ESP_OK || bytes_read < 2 * sizeof(int32_t)) {
        return 0;
    }

    const int frames = std::min(
        samples,
        static_cast<int>(bytes_read / (2 * sizeof(int32_t))));

    for (int i = 0; i < frames; ++i) {
        // INMP441 24-bit sample is in the upper bits of each 32-bit slot.
        const int16_t left = static_cast<int16_t>(raw[2 * i] >> 16);
        const int16_t right = static_cast<int16_t>(raw[2 * i + 1] >> 16);

        // Xiaozhi's speech pipeline remains mono; use left channel.
        dest[i] = left;

        left_ring_[ring_write_index_] = left;
        right_ring_[ring_write_index_] = right;
        ring_write_index_ =
            (ring_write_index_ + 1) % kTdoaWindowFrames;

        if (ring_frames_ < kTdoaWindowFrames) {
            ++ring_frames_;
        }
    }

    if (frames < samples) {
        std::memset(dest + frames, 0,
                    (samples - frames) * sizeof(int16_t));
    }

    AnalyzeDirection();
    return frames;
}

    void AnalyzeDirection() {
        if (ring_frames_ < kTdoaWindowFrames) {
            return;
        }

        const int64_t now_us = esp_timer_get_time();
        if (last_tdoa_update_us_ != 0 &&
            now_us - last_tdoa_update_us_ <
                static_cast<int64_t>(kTdoaUpdateMs) * 1000) {
            return;
        }
        last_tdoa_update_us_ = now_us;

        double left_mean = 0.0;
        double right_mean = 0.0;
        for (size_t i = 0; i < kTdoaWindowFrames; ++i) {
            const size_t index = (ring_write_index_ + i) % kTdoaWindowFrames;
            left_mean += left_ring_[index];
            right_mean += right_ring_[index];
        }
        left_mean /= kTdoaWindowFrames;
        right_mean /= kTdoaWindowFrames;

        double left_energy_all = 0.0;
        double right_energy_all = 0.0;
        for (size_t i = 0; i < kTdoaWindowFrames; ++i) {
            const size_t index = (ring_write_index_ + i) % kTdoaWindowFrames;
            const double left = left_ring_[index] - left_mean;
            const double right = right_ring_[index] - right_mean;
            left_energy_all += left * left;
            right_energy_all += right * right;
        }

        const double left_rms =
            std::sqrt(left_energy_all / kTdoaWindowFrames);
        const double right_rms =
            std::sqrt(right_energy_all / kTdoaWindowFrames);

        // The RMS gate also acts as a simple speech/ambient-noise gate.
      // Only analyze direction when the combined average RMS is high enough.
        const double average_rms = std::sqrt(
            (left_energy_all + right_energy_all) /
            (2.0 * kTdoaWindowFrames));

        auto hide_after_silence = [this, now_us]() {
            if (last_voice_us_ != 0 &&
                now_us - last_voice_us_ >= kSilenceTimeoutUs &&
                !displayed_direction_.empty()) {
                displayed_direction_.clear();
                PublishDirection("", false);
            }
        };
	ESP_LOGI(TAG, "average_rms: %.1f", average_rms);

        if (average_rms < kDirectionRmsThreshold) {
            hide_after_silence();
            return;
        }

        double best_abs_correlation = -1.0;
        int best_lag = 0;

        // correlation(lag) = sum(left[n] * right[n + lag]).
        // Positive lag means right arrived later, so classify as LEFT.
        for (int lag = -kMaxLagSamples; lag <= kMaxLagSamples; ++lag) {
            double cross = 0.0;
            double left_energy = 0.0;
            double right_energy = 0.0;

            for (int n = kMaxLagSamples;
                 n < static_cast<int>(kTdoaWindowFrames) - kMaxLagSamples;
                 ++n) {
                const size_t left_index =
                    (ring_write_index_ + static_cast<size_t>(n)) %
                    kTdoaWindowFrames;
                const size_t right_index =
                    (ring_write_index_ +
                     static_cast<size_t>(n + lag + kTdoaWindowFrames)) %
                    kTdoaWindowFrames;

                const double left = left_ring_[left_index] - left_mean;
                const double right = right_ring_[right_index] - right_mean;
                cross += left * right;
                left_energy += left * left;
                right_energy += right * right;
            }

            const double denominator = std::sqrt(left_energy * right_energy);
            const double correlation =
                denominator > 0.0 ? cross / denominator : 0.0;

            correlations_[lag + kMaxLagSamples] = correlation;
            const double absolute_correlation = std::fabs(correlation);
            if (absolute_correlation > best_abs_correlation) {
                best_abs_correlation = absolute_correlation;
                best_lag = lag;
            }
        }

        if (best_abs_correlation < kMinCorrelation) {
		 hide_after_silence();
            return;
        }

        double fractional_offset = 0.0;
        if (best_lag > -kMaxLagSamples && best_lag < kMaxLagSamples) {
            const double left =
                std::fabs(correlations_[best_lag - 1 + kMaxLagSamples]);
            const double center =
                std::fabs(correlations_[best_lag + kMaxLagSamples]);
            const double right =
                std::fabs(correlations_[best_lag + 1 + kMaxLagSamples]);
            const double denominator = left - 2.0 * center + right;

            if (std::fabs(denominator) > 1e-9) {
                fractional_offset =
                    0.5 * (left - right) / denominator;
                fractional_offset =
                    std::clamp(fractional_offset, -0.5, 0.5);
            }
        }

        const double lag_samples = best_lag + fractional_offset;
        const double delay_seconds = lag_samples / input_sample_rate_;
        double sine_angle =
            kSpeedOfSoundMps * delay_seconds / kMicSpacingM;
        sine_angle = std::clamp(sine_angle, -1.0, 1.0);

        const double angle_deg =
            std::asin(sine_angle) * 180.0 / M_PI;

        const char* direction =
            angle_deg > kCenterAngleDeg ? "LEFT" :
            angle_deg < -kCenterAngleDeg ? "RIGHT" : "CENTER";

	last_voice_us_ = now_us;
        // Do not redraw the same direction every second.
        if (displayed_direction_ != direction) {
            displayed_direction_ = direction;
            PublishDirection(displayed_direction_, true);
            ESP_LOGI(TAG, "TDOA direction: %s, angle=%.1f, corr=%.2f",
                     direction, angle_deg, best_abs_correlation);
        }
}

    void PublishDirection(const std::string& direction, bool visible) {
        Application::GetInstance().Schedule([direction, visible]() {
            auto* display = static_cast<LcdDisplay*>(
                Board::GetInstance().GetDisplay());
            if (display != nullptr) {
                display->SetDirectionIndicator(direction.c_str(), visible);
            }
        });
    }
};

class CompactWifiBoardLCD : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_;
    EspVideo* camera_;
    EnvironmentSensors* environment_sensors_ = nullptr;
    static constexpr size_t kTdoaWindowFrames = 2048;
    static constexpr int kMaxLagSamples = 8;
    static constexpr uint32_t kTdoaUpdateMs = 1000;
    static constexpr int64_t kSilenceTimeoutUs = 3000000;
    static constexpr double kMicSpacingM = 0.05;
    static constexpr double kSpeedOfSoundMps = 343.0;
    static constexpr double kMinRms = 300.0;
    static constexpr double kMinCorrelation = 0.20;
    static constexpr double kCenterAngleDeg = 20.0;

    std::array<int16_t, kTdoaWindowFrames> left_ring_{};
    std::array<int16_t, kTdoaWindowFrames> right_ring_{};
    std::array<double, kMaxLagSamples * 2 + 1> correlations_{};

    size_t ring_write_index_ = 0;
    size_t ring_frames_ = 0;
    int64_t last_tdoa_update_us_ = 0;
    int64_t last_voice_us_ = 0;
    std::string displayed_direction_;


    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeCamera() {
        esp_video_init_usb_uvc_config_t usb_uvc_config = {
            .uvc = {
                .uvc_dev_num = 1,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
            .usb = {
                .init_usb_host_lib = true,
                .task_stack = 4096,
                .task_priority = 5,
                .task_affinity = -1,
            },
        };

        esp_video_init_config_t video_config = {
            .usb_uvc = &usb_uvc_config,
        };

        camera_ = new EspVideo(video_config);
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
        environment_sensors_ = new EnvironmentSensors();

        McpServer::GetInstance().AddTool(
            "self.environment.read_sensors",
            "读取 AHT20 温湿度、BMP280 温度/气压/估算海拔、AGS10 TVOC 和 GUVA-S12SD UV估算值。用户询问环境数据时调用。",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (environment_sensors_ == nullptr) {
                    return std::string("环境传感器服务未初始化");
                }
                return environment_sensors_->ReadJson();
            });
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
	InitializeCamera();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        //static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
         //   AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        static StereoMicNoAudioCodec audio_codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
            AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK,
            AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    virtual Camera* GetCamera() override {
        return camera_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
