#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <stdio.h>
#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "config.h"
#include "custom_lcd_display.h"
#include "lvgl.h"
#include "mcp_server.h"
#include "wifi_board.h"

#include "axp2101.h"
#include "codecs/box_audio_codec.h"
#include "i2c_device.h"
#include "power_save_timer.h"
#include "settings.h"

#define TAG "waveshare_epaper_3_97"

// Built with --language zh-CN (Chinese UI, wake word 你好小智), see .github/workflows/build.yml
#define SHTC3_I2C_ADDR 0x70
// The sensor sits next to the ESP32 and reads warm; Waveshare's demo subtracts 4°C.
#define SHTC3_TEMP_OFFSET 4.0f

class Pmic : public Axp2101 {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110);  // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);   // hold 4s to power off

        // Disable All DCs but DC1
        WriteReg(0x80, 0x01);
        // Disable All LDOs
        WriteReg(0x90, 0x00);
        WriteReg(0x91, 0x00);

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO1 to 3.3V
        WriteReg(0x92, (3300 - 500) / 100);
        // Set ALDO2 to 3.3V
        WriteReg(0x93, (3300 - 500) / 100);
        // Set ALDO3 to 3.3V
        WriteReg(0x94, (3300 - 500) / 100);

        // Enable ALDO1、ALDO2、ALDO2
        WriteReg(0x90, 0x07);

        WriteReg(0x64, 0x03);  // CV charger voltage setting to 4.2V

        WriteReg(0x61, 0x02);  // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x08);  // set Main battery charger current to 400mA ( 0x08-200mA,
                               // 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01);  // set Main battery term charge current to 25mA
    }
};

class WaveshareEsp32s3ePaper3inch97 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_ = nullptr;
    Button boot_button_;
    Button pwr_button_;
    CustomEpdDisplay* display_;
    i2c_master_dev_handle_t shtc3_dev_ = nullptr;
    PowerSaveTimer* power_save_timer_;

    void InitializePowerSaveTimer() {
        // On battery only: power off after 20 minutes idle (wake word works until then).
        // When powered over USB the timer is disabled, so the device never powers off.
        power_save_timer_ = new PowerSaveTimer(-1, 100, 20 * 60);
        power_save_timer_->OnShutdownRequest([this]() {
            GetDisplay()->SetChatMessage("system", "OFF");
            vTaskDelay(pdMS_TO_TICKS(1000));
            pmic_->PowerOff();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeI2c() {
        i2c_master_bus_config_t i2c_bus_cfg = {};
        i2c_bus_cfg.i2c_port = (i2c_port_t)0;
        i2c_bus_cfg.sda_io_num = AUDIO_CODEC_I2C_SDA_PIN;
        i2c_bus_cfg.scl_io_num = AUDIO_CODEC_I2C_SCL_PIN;
        i2c_bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt = 7;
        i2c_bus_cfg.intr_priority = 0;
        i2c_bus_cfg.trans_queue_depth = 0;
        i2c_bus_cfg.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        if (i2c_master_probe(i2c_bus_, SHTC3_I2C_ADDR, 100) == ESP_OK) {
            i2c_device_config_t dev_cfg = {};
            dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_cfg.device_address = SHTC3_I2C_ADDR;
            dev_cfg.scl_speed_hz = 100000;
            if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &shtc3_dev_) != ESP_OK) {
                shtc3_dev_ = nullptr;
            }
        }
        ESP_LOGI(TAG, "SHTC3 %s", shtc3_dev_ ? "found" : "not found");
    }

    static uint8_t Shtc3Crc(const uint8_t* data) {
        uint8_t crc = 0xFF;
        for (int i = 0; i < 2; i++) {
            crc ^= data[i];
            for (int b = 0; b < 8; b++) {
                crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
            }
        }
        return crc;
    }

    static esp_err_t Shtc3Command(i2c_master_dev_handle_t dev, uint16_t cmd) {
        uint8_t buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
        return i2c_master_transmit(dev, buf, 2, 100);
    }

    bool ReadShtc3(float& temperature, float& humidity) {
        if (shtc3_dev_ == nullptr) {
            return false;
        }
        Shtc3Command(shtc3_dev_, 0x3517);  // wake up
        vTaskDelay(pdMS_TO_TICKS(1));
        bool ok = false;
        if (Shtc3Command(shtc3_dev_, 0x7866) == ESP_OK) {  // measure, T first, no clock stretching
            vTaskDelay(pdMS_TO_TICKS(20));
            uint8_t d[6] = {};
            if (i2c_master_receive(shtc3_dev_, d, sizeof(d), 100) == ESP_OK &&
                Shtc3Crc(d) == d[2] && Shtc3Crc(d + 3) == d[5]) {
                uint16_t raw_t = (d[0] << 8) | d[1];
                uint16_t raw_h = (d[3] << 8) | d[4];
                temperature = 175.0f * raw_t / 65536.0f - 45.0f - SHTC3_TEMP_OFFSET;
                humidity = 100.0f * raw_h / 65536.0f;
                ok = true;
            }
        }
        Shtc3Command(shtc3_dev_, 0xB098);  // sleep
        return ok;
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            // During startup (before connected), pressing BOOT button enters Wi-Fi config mode
            // without reboot
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        pwr_button_.OnClick([this]() {
            GetDisplay()->SetChatMessage("system", "OFF");
            vTaskDelay(pdMS_TO_TICKS(1000));
            pmic_->PowerOff();
        });
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.disp.network", "重新配网", PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               EnterWifiConfigMode();
                               return true;
                           });

        mcp_server.AddTool(
            "self.sensor.get_indoor_climate",
            "读取板载传感器测得的室内温度(摄氏度)和湿度(%)，传感器靠近芯片，数值仅供参考。用户问“现在几度/室温多少/冷不冷/湿度多少”时调用。",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                float t, h;
                if (!ReadShtc3(t, h)) {
                    return std::string("{\"error\":\"sensor not available\"}");
                }
                char buf[96];
                snprintf(buf, sizeof(buf), "{\"temperature_c\":%.1f,\"humidity_percent\":%.0f}", t, h);
                return std::string(buf);
            });

        mcp_server.AddTool(
            "self.disp.set_rotation",
            "设置墨水屏显示方向，设置后设备会自动重启生效。rotation: 270=竖屏(默认, Type-C口朝下), "
            "90=竖屏(倒过来), 0=横屏, 180=横屏(倒过来)。用户说“竖屏”“横屏”“屏幕倒了/上下颠倒/旋转屏幕”时调用。",
            PropertyList({Property("rotation", kPropertyTypeInteger, 0, 270)}),
            [](const PropertyList& properties) -> ReturnValue {
                int rotation = properties["rotation"].value<int>();
                rotation = ((rotation + 45) / 90 % 4) * 90;
                {
                    Settings settings("display", true);
                    settings.SetInt("rotation", rotation);
                }
                ESP_LOGI(TAG, "Set display rotation to %d, rebooting", rotation);
                auto& app = Application::GetInstance();
                app.Schedule([&app]() {
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    app.Reboot();
                });
                return true;
            });
    }

    void InitializeEpdDisplay() {
        custom_epd_spi_t epd_spi_data = {};
        epd_spi_data.cs = EPD_CS_PIN;
        epd_spi_data.dc = EPD_DC_PIN;
        epd_spi_data.rst = EPD_RST_PIN;
        epd_spi_data.busy = EPD_BUSY_PIN;
        epd_spi_data.mosi = EPD_MOSI_PIN;
        epd_spi_data.scl = EPD_SCK_PIN;
        epd_spi_data.spi_host = EPD_SPI_NUM;
        epd_spi_data.buffer_len = 48000;
        display_ = new CustomEpdDisplay(NULL, NULL, EXAMPLE_LCD_WIDTH, EXAMPLE_LCD_HEIGHT,
                                        DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                        DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY, epd_spi_data);
    }

public:
    WaveshareEsp32s3ePaper3inch97()
        : boot_button_(BOOT_BUTTON_GPIO), pwr_button_(VBAT_PWR_GPIO, 1) {
        InitializePowerSaveTimer();
        InitializeI2c();
        InitializeAxp2101();
        InitializeButtons();
        InitializeTools();
        InitializeEpdDisplay();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(WaveshareEsp32s3ePaper3inch97);