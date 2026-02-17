#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "telemetry.h"
#include "radio.h"
#include "motors.h"
#include "types.h"
#include "html_page_gz.h"

const char* ssid = "Drone_ESP32";
const char* password = "password123";

DroneState* drone_data;
AsyncWebServer server(80);

void telemetryTask(void * parameter) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html_page_gz, html_page_gz_len);
        response->addHeader("Content-Encoding", "gzip");
        request->send(response);
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        // Attitude
        response->printf("{\"ar\":%.2f,\"ap\":%.2f,\"ay\":%.2f,",
            drone_data->angle_roll, drone_data->angle_pitch, drone_data->angle_yaw);
        // Radio
        response->printf("\"r1\":%d,\"r2\":%d,\"r3\":%d,\"r4\":%d,\"r5\":%d,\"r6\":%d,",
            drone_data->channel_1, drone_data->channel_2, drone_data->channel_3,
            drone_data->channel_4, drone_data->channel_5, drone_data->channel_6);
        // Timing
        response->printf("\"lt\":%lu,\"mr\":%lu,\"mi\":%lu,\"ci\":%lu,\"mp\":%lu,",
            drone_data->loop_time, drone_data->max_time_radio, drone_data->max_time_imu,
            drone_data->current_time_imu, drone_data->max_time_pid);
        // Yaw PID
        response->printf("\"gy\":%.2f,\"poy\":%.2f,",
            drone_data->gyro_yaw_input, drone_data->pid_output_yaw);
        // Accel + Battery
        response->printf("\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"vb\":%.1f,",
            drone_data->acc_x, drone_data->acc_y, drone_data->acc_z, drone_data->voltage_bat);
        // Alt IMU
        response->printf("\"alt_ar\":%.2f,\"alt_ap\":%.2f,",
            drone_data->alt_angle_roll, drone_data->alt_angle_pitch);
        response->printf("\"alt_ax\":%.2f,\"alt_ay\":%.2f,\"alt_az\":%.2f,",
            drone_data->alt_acc_x, drone_data->alt_acc_y, drone_data->alt_acc_z);
        response->printf("\"alt_gr\":%.2f,\"alt_gp\":%.2f,\"alt_gy\":%.2f,\"alt_ayw\":%.1f,",
            drone_data->alt_gyro_roll, drone_data->alt_gyro_pitch, drone_data->alt_gyro_yaw,
            drone_data->alt_angle_yaw);
        // LiDAR + Altitude
        response->printf("\"ld\":%.1f,\"lv\":%.1f,\"asp\":%.1f,\"poa\":%.2f,\"tc\":%d,",
            drone_data->lidar_distance, drone_data->lidar_velocity,
            drone_data->alt_setpoint, drone_data->pid_output_alt, drone_data->throttle_command);
        // Optical Flow
        response->printf("\"fvx\":%.1f,\"fvy\":%.1f,\"fq\":%d,\"frx\":%.1f,\"fry\":%.1f,",
            drone_data->flow_velocity_x, drone_data->flow_velocity_y,
            drone_data->flow_quality, drone_data->flow_raw_x, drone_data->flow_raw_y);
        // Position Hold
        response->printf("\"pex\":%.1f,\"pey\":%.1f,\"psx\":%.1f,\"psy\":%.1f,",
            drone_data->pos_est_x, drone_data->pos_est_y,
            drone_data->pos_setpoint_x, drone_data->pos_setpoint_y);
        response->printf("\"popr\":%.2f,\"popp\":%.2f,",
            drone_data->pid_output_pos_roll, drone_data->pid_output_pos_pitch);
        // EKF + Quality Fade
        response->printf("\"evx\":%.1f,\"evy\":%.1f,\"qf\":%.2f,",
            drone_data->ekf_vel_x, drone_data->ekf_vel_y, drone_data->quality_fade);
        // Status
        response->printf("\"ato\":%s,\"fm\":%d}",
            drone_data->auto_takeoff_active ? "true" : "false", (int)drone_data->current_mode);
        request->send(response);
    });
    
    server.on("/reset_max", HTTP_GET, [](AsyncWebServerRequest *request){
        drone_data->max_time_radio = 0;
        drone_data->max_time_imu = 0;
        drone_data->max_time_pid = 0;
        request->send(200, "text/plain", "OK");
    });

    server.on("/get_pid", HTTP_GET, [](AsyncWebServerRequest *request){
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        response->printf("{\"ppr\":%.2f,\"ipr\":%.4f,\"dpr\":%.2f,",
            drone_data->p_pitch_roll, drone_data->i_pitch_roll, drone_data->d_pitch_roll);
        response->printf("\"py\":%.2f,\"iy\":%.4f,\"dy\":%.2f,",
            drone_data->p_yaw, drone_data->i_yaw, drone_data->d_yaw);
        response->printf("\"ffpr\":%.2f,\"ffy\":%.2f,\"pl\":%.2f,\"ph\":%.2f,",
            drone_data->ff_pitch_roll, drone_data->ff_yaw, drone_data->p_level, drone_data->p_heading);
        response->printf("\"tr\":%.2f,\"tp\":%.2f,",
            drone_data->trim_roll, drone_data->trim_pitch);
        response->printf("\"pa\":%.2f,\"ia\":%.4f,\"da\":%.2f,\"hov\":%d,\"als\":%.2f,",
            drone_data->p_alt, drone_data->i_alt, drone_data->d_alt, drone_data->hover_throttle,
            drone_data->alt_smooth);
        response->printf("\"pp\":%.2f,\"ip\":%.4f,\"dp\":%.2f,\"ps\":%.2f}",
            drone_data->p_pos, drone_data->i_pos, drone_data->d_pos, drone_data->pos_smooth);
        request->send(response);
    });

    server.on("/set_pid", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasParam("ppr")) drone_data->p_pitch_roll = request->getParam("ppr")->value().toFloat();
        if(request->hasParam("ipr")) drone_data->i_pitch_roll = request->getParam("ipr")->value().toFloat();
        if(request->hasParam("dpr")) drone_data->d_pitch_roll = request->getParam("dpr")->value().toFloat();
        if(request->hasParam("py")) drone_data->p_yaw = request->getParam("py")->value().toFloat();
        if(request->hasParam("iy")) drone_data->i_yaw = request->getParam("iy")->value().toFloat();
        if(request->hasParam("dy")) drone_data->d_yaw = request->getParam("dy")->value().toFloat();
        if(request->hasParam("pl")) drone_data->p_level = request->getParam("pl")->value().toFloat();
        if(request->hasParam("ph")) drone_data->p_heading = request->getParam("ph")->value().toFloat();
        if(request->hasParam("ffpr")) drone_data->ff_pitch_roll = request->getParam("ffpr")->value().toFloat();
        if(request->hasParam("ffy")) drone_data->ff_yaw = request->getParam("ffy")->value().toFloat();
        if(request->hasParam("tr")) drone_data->trim_roll = request->getParam("tr")->value().toFloat();
        if(request->hasParam("tp")) drone_data->trim_pitch = request->getParam("tp")->value().toFloat();
        if(request->hasParam("pa")) drone_data->p_alt = request->getParam("pa")->value().toFloat();
        if(request->hasParam("ia")) drone_data->i_alt = request->getParam("ia")->value().toFloat();
        if(request->hasParam("da")) drone_data->d_alt = request->getParam("da")->value().toFloat();
        if(request->hasParam("hov")) drone_data->hover_throttle = request->getParam("hov")->value().toInt();
        if(request->hasParam("als")) drone_data->alt_smooth = request->getParam("als")->value().toFloat();
        if(request->hasParam("pp")) drone_data->p_pos = request->getParam("pp")->value().toFloat();
        if(request->hasParam("ip")) drone_data->i_pos = request->getParam("ip")->value().toFloat();
        if(request->hasParam("dp")) drone_data->d_pos = request->getParam("dp")->value().toFloat();
        if(request->hasParam("ps")) drone_data->pos_smooth = request->getParam("ps")->value().toFloat();

        request->send(200, "text/plain", "OK");
    });

    server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasParam("m") && request->hasParam("val")) {
            int m = request->getParam("m")->value().toInt();
            int val = request->getParam("val")->value().toInt();
            if(drone_data->current_mode == MODE_SAFE || drone_data->current_mode == MODE_WEB_TEST) {
                drone_data->current_mode = MODE_WEB_TEST;
                for(int i=1; i<=4; i++) if(i!=m) drone_data->web_test_vals[i] = 1000;
                if(m>=1 && m<=4) drone_data->web_test_vals[m] = val;
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
        drone_data->current_mode = MODE_SAFE;
        motors_stop();
        for(int i=1; i<=4; i++) drone_data->web_test_vals[i] = 1000;
        request->send(200, "text/plain", "STOPPED");
    });

    // Favicon : eviter une requete 404 lourde a chaque refresh
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(204);
    });

    // Requetes inconnues : repondre vite pour liberer la connexion
    server.onNotFound([](AsyncWebServerRequest *request){
        request->send(404, "text/plain", "Not found");
    });

    server.begin();
    vTaskDelete(NULL);
}

void start_telemetry_task(DroneState* drone_ptr) {
    drone_data = drone_ptr;
    // Deplacer sur Core 1 pour ne pas interferer avec l'IMU principal sur Core 0
    // Stack augmentee pour le WiFi qui est gourmand en memoire
    xTaskCreatePinnedToCore(telemetryTask, "WifiTask", 16384, NULL, 1, NULL, 1);
}
