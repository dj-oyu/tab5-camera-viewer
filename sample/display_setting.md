# tab5 display init
```c
esp_err_t bsp_display_new_with_handles(const bsp_display_config_t* config, bsp_lcd_handles_t* ret_handles)
{
    esp_err_t ret                     = ESP_OK;
    esp_lcd_panel_io_handle_t io      = NULL;
    esp_lcd_panel_handle_t disp_panel = NULL;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    ESP_RETURN_ON_ERROR(bsp_enable_dsi_phy_power(), TAG, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_config   = {
          .bus_id             = 0,
          .num_data_lanes     = BSP_LCD_MIPI_DSI_LANE_NUM,
          .phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
          .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), TAG, "New DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits    = 8,  // according to the LCD spec
        .lcd_param_bits  = 8,  // according to the LCD spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, TAG, "New panel IO failed");

#if defined(LCD_MIPI_DSI_USE_ILI9881C) && !defined(LCD_MIPI_DSI_USE_ST7703)
    ESP_LOGI(TAG, "Install LCD driver of ili9881c");
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel    = 0,
        .dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 60,  // 720*1280 RGB24 60Hz RGB24 // 80,
        .pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs            = 1,
        .video_timing =
            {
                .h_size            = BSP_LCD_H_RES,
                .v_size            = BSP_LCD_V_RES,
                .hsync_back_porch  = 140,
                .hsync_pulse_width = 40,
                .hsync_front_porch = 40,
                .vsync_back_porch  = 20,
                .vsync_pulse_width = 4,
                .vsync_front_porch = 20,
            },
        .flags.use_dma2d = true,
    };

    ili9881c_vendor_config_t vendor_config = {
        .init_cmds      = tab5_lcd_ili9881c_specific_init_code_default,
        .init_cmds_size = sizeof(tab5_lcd_ili9881c_specific_init_code_default) /
                          sizeof(tab5_lcd_ili9881c_specific_init_code_default[0]),
        .mipi_config =
            {
                .dsi_bus    = mipi_dsi_bus,
                .dpi_config = &dpi_config,
                .lane_num   = 2,
            },
    };

    const esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 16,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .reset_gpio_num = -1,
        .vendor_config  = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9881c(io, &lcd_dev_config, &disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(disp_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(disp_panel));
    //  ESP_ERROR_CHECK(esp_lcd_panel_mirror(disp_panel, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(disp_panel, true));

#elif defined(LCD_MIPI_DSI_USE_ST7703) && !defined(LCD_MIPI_DSI_USE_ILI9881C)
    ESP_LOGI(TAG, "Install LCD driver of ST7703");
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 60,                       // LCD_MIPI_DSI_DPI_CLK_MHZ_ST7703,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,  // LCD_COLOR_PIXEL_FORMAT_RGB888,
        .num_fbs = 1,
        .video_timing =
            {
                .h_size = BSP_LCD_H_RES,  // lcd_param.width,
                .v_size = BSP_LCD_V_RES,  // lcd_param.height,
                .hsync_back_porch = 40,
                .hsync_pulse_width = 10,
                .hsync_front_porch = 40,
                .vsync_back_porch = 16,
                .vsync_pulse_width = 4,
                .vsync_front_porch = 16,
            },
        //.flags.use_dma2d = true, // ??? 开启后需要等待 previous draw 完成
    };

    st7703_vendor_config_t vendor_config = {
        .flags.use_mipi_interface = 1,
        .mipi_config =
            {
                .dsi_bus = mipi_dsi_bus,
                .dpi_config = &dpi_config,
            },
    };
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 16,  // 24,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .reset_gpio_num = -1,
        .vendor_config = &vendor_config,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7703(io, &lcd_dev_config, &disp_panel), err, TAG,
                      "New LCD panel EK79007 failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");
#endif

    /* Return all handles */
    ret_handles->io           = io;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel        = disp_panel;
    ret_handles->control      = NULL;

    ESP_LOGI(TAG, "Display initialized with resolution %dx%d", BSP_LCD_H_RES, BSP_LCD_V_RES);

    return ret;

err:
    if (disp_panel) {
        esp_lcd_panel_del(disp_panel);
    }
    if (io) {
        esp_lcd_panel_io_del(io);
    }
    if (mipi_dsi_bus) {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
    }
    return ret;
}
```

# フレームバッファ

フレームバッファを2枚用意してデコーダの出力とLCDへの転送を交互に行うことにしましょう。
デコーダはLCDに表示されていない方のフレームバッファに画像を出力して、出力し終わってから表示するフレームバッファを切り替えることでデコード途中の画像が表示されるのを防ぐ感じです。
ちなみにこれ以降の実装はドライバを直接いじる必要があり、M5Unifiedが使えないため、ドライバは自作しましょう。

esp_lcd_dpi_panel_config_t の num_fbs を2にすることで、ドライバ内で確保されるフレームバッファを2枚にすることができます。M5Tab5-UserDemoの実装だとここですね。

あとは esp_lcd_dpi_panel_get_frame_buffer を使うとフレームバッファのポインタを2枚分取得できます。

これを使ってレンダリングする際ですが、フレームバッファの切り替えには esp_lcd_panel_draw_bitmap を使います。
esp_lcd_panel_draw_bitmapは実装を見ると、

color_data に渡されたポインタがフレームバッファの範囲内ならcache flush & フレームバッファ切り替え
そうでなければ color_data をフレームバッファにコピー
という感じの挙動なので、とりあえずこいつに切り替え先のフレームバッファのポインタを渡しておけば大丈夫です。

# リフレッシュレート

リフレッシュレートの計算式は

dpi_clock_freq_mhz / (hsync_pulse_width + hsync_front_porch + hsync_back_porch + h_size) / (vsync_pulse_width + vsync_front_porch + vsync_back_porch + v_size)

なので、Tab5-UserDemoと同じパラメータを使った場合は 600000000 / (40 + 40 + 140 + 720) / (4 + 20 + 20 + 1280) := 48.2 となって48.2Hzで動作していることがわかります。
これはちょっと遅いですね。60fps描画しようとしているところに間に合っていないのもそうなのですが、そもそもILI9881C自体の推奨値が50~60fpsなので、48.2はそれを下回っています。
そこで、60fpsになるように dpi_clock_freq_mhz を上げます。60 * (40 + 40 + 140 + 720) * (4 + 20 + 20 + 1280) == 74.6736 なので、 dpi_clock_freq_mhz を75にしてみましょう。あと、そうするとバスの転送速度が足りなくなるので、 lane_bit_rate_mbps も800くらいに上げておきます。

なお、このプロジェクトで再生しているmjpeg動画は30fps

[source](https://ideal-reality.com/post/0002#HW-Jpeg-Codec)
