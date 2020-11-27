
#include <esp_camera.h>
//#include "xclk.h"

// Forward declare cam clock stop/start. 
//esp_err_t camera_enable_out_clock(camera_config_t *config);
//void camera_disable_out_clock();

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32 //
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0 //
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22 //

#define SerialDebug Serial    // Nice names for Serial ports

camera_config_t cam_config;
camera_fb_t *fb; // Declare the variable for the pointer to the framebuffer

void setup() {
    SerialDebug.begin(115200);
    
    setCpuFrequencyMhz(20); // Save power by going slowly

    // Init out Camera
    cam_config.ledc_channel = LEDC_CHANNEL_0;
    cam_config.ledc_timer = LEDC_TIMER_0;
    cam_config.pin_d0 = Y2_GPIO_NUM;
    cam_config.pin_d1 = Y3_GPIO_NUM;
    cam_config.pin_d2 = Y4_GPIO_NUM;
    cam_config.pin_d3 = Y5_GPIO_NUM;
    cam_config.pin_d4 = Y6_GPIO_NUM;
    cam_config.pin_d5 = Y7_GPIO_NUM;
    cam_config.pin_d6 = Y8_GPIO_NUM;
    cam_config.pin_d7 = Y9_GPIO_NUM;
    cam_config.pin_xclk = XCLK_GPIO_NUM;
    cam_config.pin_pclk = PCLK_GPIO_NUM;
    cam_config.pin_vsync = VSYNC_GPIO_NUM;
    cam_config.pin_href = HREF_GPIO_NUM;
    cam_config.pin_sscb_sda = SIOD_GPIO_NUM;
    cam_config.pin_sscb_scl = SIOC_GPIO_NUM;
    cam_config.pin_pwdn = PWDN_GPIO_NUM;
    cam_config.pin_reset = RESET_GPIO_NUM;
    cam_config.xclk_freq_hz = 20000000;
    cam_config.pixel_format = PIXFORMAT_JPEG;
    //init with high specs to pre-allocate larger buffers
    if (psramFound()) {
      SerialDebug.println("  PS RAM Found.");
      cam_config.frame_size = FRAMESIZE_UXGA;
      cam_config.jpeg_quality = 10;
      cam_config.fb_count = 2;
    } else {
      SerialDebug.println("  Error: PS RAM Not Found.");
      cam_config.frame_size = FRAMESIZE_SVGA;
      cam_config.jpeg_quality = 12;
      cam_config.fb_count = 1;
    }

    // Overide the above for testing
    cam_config.frame_size = FRAMESIZE_VGA;
    cam_config.jpeg_quality = 10;  // Quality of JPEG output. 0-63 lower means higher quality
    cam_config.fb_count = 1;

    SerialDebug.printf("  Camera not initialised\n");
    delay(10000); // doing stuff here

    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, LOW);
    esp_camera_init(&cam_config);

    SerialDebug.printf("  Camera initialised\n");
    delay(10000); // doing stuff here

    SerialDebug.printf("  Taking picture at 80Mhz:\n");
    setCpuFrequencyMhz(80); // Camera seems to need more than 40mhz to get a picture
    //camera_enable_out_clock(&cam_config);
    delay(500);
    fb = esp_camera_fb_get(); // Get the current frame buffer
    delay(10000); // doing stuff here

    SerialDebug.printf("  Power down Camera\n");
    esp_camera_fb_return(fb);
    //camera_disable_out_clock();
    digitalWrite(PWDN_GPIO_NUM, HIGH);
    delay(10000); // doing stuff here

    SerialDebug.printf("  Back to 20Mhz\n");
    setCpuFrequencyMhz(20);
    delay(10000); // doing stuff here
    
  
}



void loop() {
  
}
