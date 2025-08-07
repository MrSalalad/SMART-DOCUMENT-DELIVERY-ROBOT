#include <lock_inferencing.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "model-parameters/model_variables.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <algorithm>
#include <WiFi.h>
#include <PubSubClient.h>

using namespace std;

// ====== Voice Recognition Pins ======
const int LED_ERROR = 18;   // Error LED (red)
const int LED_RECORD = 19;  // Recording LED (yellow)
const int LED_SUCCESS = 23; // Success LED (green)
Servo myServo;

// ====== Motor Control Pins ======
int motor1pin1 = 25;
int motor1pin2 = 26;
int motor2pin1 = 27;
int motor2pin2 = 14;

// ====== Motor Configuration ======
const float motor_rpm = 208;        // RPM at 5V
const float wheel_diameter_mm = 65.0;
const float robot_width_cm = 12.0;

// ====== WiFi Configuration ======
const char* ssid = "2HA1";
const char* password = "mk2ha113";
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// ====== Navigation Variables ======
enum Direction { NORTH, EAST, SOUTH, WEST };
Direction currentDirection;
int currentLocation;
int startingNode = 0;
int endingNode = 0;
bool unlocked = false;
bool authSuccess = false;

// ====== Voice Recognition Variables ======
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false;
static bool record_status = true;

// ====== Node Structure for Mapping ======
struct Edge {
    int neighborId;
    float cost;
};

struct Node {
    int id;
    float x, y;
    string name;
    string label;
    vector<Edge> neighbors;

    // A* metadata
    float gCost = numeric_limits<float>::infinity();
    float hCost = 0;
    float fCost = 0;
    int parentId = -1;

    void reset() {
        gCost = numeric_limits<float>::infinity();
        hCost = 0;
        fCost = 0;
        parentId = -1;
    }
};

vector<Node> allNodes;

// ====== Function Declarations ======
// Voice Recognition Functions
static void audio_inference_callback(uint32_t n_bytes);
static void capture_samples(void* arg);
static bool microphone_inference_start(uint32_t n_samples);
static bool microphone_inference_record(void);
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);
static void microphone_inference_end(void);
static int i2s_init(uint32_t sampling_rate);
static int i2s_deinit(void);
bool runVoiceInferenceCycle();

// Navigation Functions
float heuristic(const Node& a, const Node& b);
void resetGraph(vector<Node>& graph);
vector<int> aStar(int startId, int goalId, vector<Node>& graph);
int findNearestStation(int fromId, vector<Node>& graph);
void reconnect();
void callback(char* topic, uint8_t* payload, unsigned int length);
void moveForward();
void stopMotors();
void turnLeft90();
void turnRight90();
void turnBack();
float calculateDurationMs(float distance_cm, float rpm, float wheel_diameter_mm);
void moveDistance(float distance_cm);
Direction getDirectionBetween(Node a, Node b);
void rotateToDirection(Direction targetDir);
void followPath(vector<int> path);

// ====== Setup ======
void setup() {
    // Initialize LEDs
    pinMode(LED_ERROR, OUTPUT);
    pinMode(LED_RECORD, OUTPUT);
    pinMode(LED_SUCCESS, OUTPUT);
    digitalWrite(LED_ERROR, LOW);
    digitalWrite(LED_RECORD, LOW);
    digitalWrite(LED_SUCCESS, LOW);

    // Initialize Servo
    myServo.attach(13);
    myServo.write(0); // Locked position

    // Initialize Motors
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor2pin1, OUTPUT);
    pinMode(motor2pin2, OUTPUT);

    // Initialize Serial
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Starting Smart Delivery Robot");

     // Connect to WiFi
    WiFi.begin(ssid, password);
     Serial.print("Connecting to WiFi");
     while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
            
    // Connect to MQTT
    client.setServer(mqtt_server, 1883);
    reconnect();

    // đây là chỗ thêm các trạm, phòng, và ngã rẽ, gồm có id và toạ độ (x,y))
    allNodes.push_back({0, 0.0, 0.0, "Station 1", "STATION", {}}); // STATION 1: 0(0,0)
    allNodes.push_back({1, 20.0, 0.0, "Intersection 1", "INTERSECTION", {}}); // INTERSECTION 1
    allNodes.push_back({2, 40.0, 0.0, "Room 1", "ROOM", {}}); // ROOM 1
    allNodes.push_back({3, 60.0, 0.0, "Intersection 2", "INTERSECTION", {}}); // INTERSECTION 2
    allNodes.push_back({4, 80.0, 0.0, "Intersection 3", "INTERSECTION", {}}); // INTERSECTION 3
    allNodes.push_back({5, 100.0, 0.0, "Room 2", "ROOM", {}}); // ROOM 2
    allNodes.push_back({6, 20.0, 20.0, "Room 3", "ROOM", {}}); // ROOM 3
    allNodes.push_back({7, 60.0, 20.0, "Intersection 4", "INTERSECTION", {}}); // INTERSECTION 4
    allNodes.push_back({8, 80.0, 20.0, "Intersection 5", "INTERSECTION", {}}); // INTERSECTION 5
    allNodes.push_back({9, 0.0, 40.0, "Room 4", "ROOM", {}}); // ROOM 4
    allNodes.push_back({10, 20.0, 40.0, "Intersection 6", "INTERSECTION", {}}); // INTERSECTION 6
    allNodes.push_back({11, 60.0, 40.0, "Station 2", "STATION", {}}); // STATION 2
    allNodes.push_back({12, 80.0, 40.0, "Station 3", "STATION", {}}); // STATION 3

    // Create connections (undirected graph)
    // đây là chỗ thêm đường đi, lưu ý thêm 2 chiều
    allNodes[0].neighbors.push_back({1, 20.0f}); // từ node 0 đến node 1 là 40 
    allNodes[0].neighbors.push_back({9, 40.0f});
    allNodes[1].neighbors.push_back({0, 20.0f});
    allNodes[1].neighbors.push_back({2, 20.0f});
    allNodes[1].neighbors.push_back({6, 20.0f});
    allNodes[2].neighbors.push_back({1, 20.0f});
    allNodes[2].neighbors.push_back({3, 20.0f});
    allNodes[3].neighbors.push_back({2, 20.0f});
    allNodes[3].neighbors.push_back({7, 20.0f}); // 100
    allNodes[4].neighbors.push_back({5, 20.0f});
    allNodes[4].neighbors.push_back({8, 20.0f}); //100
    allNodes[5].neighbors.push_back({4, 20.0f});
    allNodes[6].neighbors.push_back({1, 20.0f});
    allNodes[6].neighbors.push_back({7, 40.0f});
    allNodes[6].neighbors.push_back({10, 20.0f});
    allNodes[7].neighbors.push_back({6, 40.0f});
    allNodes[7].neighbors.push_back({8, 20.0f});
    allNodes[7].neighbors.push_back({3, 20.0f});
    allNodes[7].neighbors.push_back({11, 20.0f});
    allNodes[8].neighbors.push_back({7, 20.0f});
    allNodes[8].neighbors.push_back({4, 20.0f}); // 100
    allNodes[9].neighbors.push_back({0, 20.0f});
    allNodes[9].neighbors.push_back({10, 20.0f});
    allNodes[10].neighbors.push_back({9, 20.0f});
    allNodes[10].neighbors.push_back({6, 20.0f});
    allNodes[11].neighbors.push_back({7, 20.0f});
    allNodes[11].neighbors.push_back({12, 20.0f});
    allNodes[12].neighbors.push_back({11, 20.0f});

    // Initialize Voice Recognition
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));
    
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Could not allocate audio buffer\n");
        while(1);
    }

    // Initial robot state
    currentDirection = EAST;
    currentLocation = 0;

    // Set MQTT callback
    client.setCallback(callback);
}

// ====== Main Loop ======
void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

// ====== Voice Recognition Functions ======
bool runVoiceInferenceCycle() {
    // Start recording
    bool m = microphone_inference_record();

    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return false;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        return false;
    }

    // Check for correct command (class index 1)
    if (result.classification[1].value > result.classification[0].value && 
        result.classification[1].value > result.classification[2].value) {
        return true;
    } else {
        return false;
    }
}

// ====== Navigation Functions ======
void callback(char* topic, uint8_t* payload, unsigned int length) {
    char jsonStr[100];
    memcpy(jsonStr, payload, length);
    jsonStr[length] = '\0';

    StaticJsonDocument<200> doc;
    deserializeJson(doc, jsonStr);

    // startingNode = doc["source"];
    // endingNode = doc["destination"];

    const char* sourceName = doc["source"];
    const char* destinationName = doc["destination"];
    unlocked = false;

    // Tìm ID tương ứng với tên node
    int startingNode = -1;
    int endingNode = -1;

    for (const auto& node : allNodes) {
        if (node.name == sourceName) {
            startingNode = node.id;
        }
        if (node.name == destinationName) {
            endingNode = node.id;
        }
    }

    if (startingNode == -1 || endingNode == -1) {
        Serial.println("❌ Không tìm thấy node phù hợp với tên đã cung cấp.");
        return;
    }

    Serial.print("Starting Node: ");
    Serial.println(startingNode);
    Serial.print("Ending Node: ");
    Serial.println(endingNode);

    // Execute delivery path
    vector<int> pathToSource = aStar(currentLocation, startingNode, allNodes);
    followPath(pathToSource);
    delay(5000);

    vector<int> pathToDestination = aStar(currentLocation, endingNode, allNodes);
    followPath(pathToDestination);
    delay(500);

    // ==== VOICE AUTH ====
    // Chỉ làm nếu chưa unlock
    if (!unlocked) {
        Serial.println("🎤 Bắt đầu xác thực giọng nói...");
        const unsigned long authStart = millis();
        const unsigned long authTimeout = 15000; // 15 giây tối đa để xác thực
        bool authSuccess = false;
        int attempts = 0;
        const int maxAttempts = 10;

        while (attempts < maxAttempts) {
            attempts++;
            Serial.print("Attempt voice auth #"); 
            Serial.println(attempts);

            // Bắt đầu ghi âm, bật đèn ghi âm
            digitalWrite(LED_RECORD, HIGH);

            if (runVoiceInferenceCycle()) {
                digitalWrite(LED_RECORD, LOW);
                authSuccess = true;
                break;
            } else {
                Serial.println("Xác thực thất bại, thử lại...");

                // Nhấp nháy đèn lỗi 3 lần
                for (int i = 0; i < 3; i++) {
                    digitalWrite(LED_ERROR, HIGH);
                    delay(500);
                    digitalWrite(LED_ERROR, LOW);
                    delay(500);
                }

                delay(3000); // nghỉ giữa các lần thử
            }

            // Tắt đèn ghi âm sau mỗi lần thử
            digitalWrite(LED_RECORD, LOW);
        }
    
        if (authSuccess) {
            unlocked = true;
            Serial.println("✅ Auth thành công, mở khóa");

            // Nhấp nháy đèn thành công 3 lần
            for (int i = 0; i < 3; i++) {
                digitalWrite(LED_SUCCESS, HIGH);
                delay(500);
                digitalWrite(LED_SUCCESS, LOW);
                delay(500);
            }

            // Mở khóa servo
            myServo.write(90); // Unlock
            delay(5000); // Giữ servo mở khóa trong 5 giây
            myServo.write(0); // Quay lại vị trí cũ
        }
    }

    // Thực hiện tìm đường về
    int nearestStationId = findNearestStation(currentLocation, allNodes);
    if (nearestStationId != -1) {
        vector<int> pathToStation = aStar(currentLocation, nearestStationId, allNodes);
        followPath(pathToStation);
    }
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("esp32-client")) {
            client.subscribe("transfer/documents");
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5s");
            delay(5000);
        }
    }
}

// ====== CÁC HÀM DI CHUYỂN CƠ BẢN ======

void moveForward() {
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin2, HIGH);
}

void stopMotors() { 
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void turnLeft90() {
  Serial.println("Turn left");
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  delay(406); // thời gian để rẽ tới góc 90 độ
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
}

void turnRight90() {
  Serial.println("Turn right");
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor2pin2, HIGH);
  delay(809); // thời gian để rẽ tới góc 90 độ
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void turnBack() {
  Serial.println("Turn back");
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  delay(765); // thời gian để rẽ tới góc 90 độ
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
}

// ====== HÀM ĐI THẲNG THEO CM ======

float calculateDurationMs(float distance_cm, float rpm, float wheel_diameter_mm) {
  // Tính chu vi bánh xe
  float wheel_circumference_cm = 3.1416 * (wheel_diameter_mm / 10.0);  // mm → cm

  // Tính số vòng mỗi giây
  float rps = motor_rpm / 60.0;

  // Tính tốc độ robot: cm/giây
  float speed_cm_per_sec = wheel_circumference_cm * rps;

  // Thời gian cần chạy (ms)
  return (distance_cm / speed_cm_per_sec) * 1000.0;  // ms
}

void moveDistance(float distance_cm) {
  float duration_ms = calculateDurationMs(distance_cm, motor_rpm, wheel_diameter_mm);
  
  Serial.print("Move forward ");
  Serial.println(distance_cm);

  moveForward();
  delay(duration_ms);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin2, LOW);
  // stopMotors();
}

// ====== Path Following Functions ======
Direction getDirectionBetween(Node a, Node b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return (abs(dx) > abs(dy)) ? (dx > 0 ? EAST : WEST) : (dy > 0 ? NORTH : SOUTH);
}

void rotateToDirection(Direction targetDir) {
    int diff = (targetDir - currentDirection + 4) % 4;
    if (diff == 1) turnRight90();
    else if (diff == 3) turnLeft90();
    else if (diff == 2) turnBack();
    currentDirection = targetDir;
}

void followPath(vector<int> path) {
  Serial.println("Path:");
  for (int i = 0; i < path.size(); ++i) {
    Serial.print(path[i]);
    if (i < path.size() - 1) Serial.print(" -> ");
  }
  Serial.println();

  for (int i = 0; i < path.size() - 1; ++i) {
    Serial.println(path[i]);
    Node& current = allNodes[path[i]];
    Node& next = allNodes[path[i + 1]];

    Direction nextDir = getDirectionBetween(current, next);
    rotateToDirection(nextDir);
    delay(500);

    // Tính khoảng cách cần đi
    float dx = next.x - current.x;
    float dy = next.y - current.y;
    float distance = sqrt(dx * dx + dy * dy);

    moveDistance(distance);
    delay(500); // nghỉ chút giữa các bước đi

    const char* name = allNodes[path[i]].name.c_str();
    char payload[100];  // đủ lớn cho tên dài
    snprintf(payload, sizeof(payload), "{\"name\":\"%s\"}", name);
    client.publish("delivery/location", payload);
  }
  currentLocation = path.back();
  Node& finalNode = allNodes[currentLocation];

  char payload[100]; // Tăng buffer đủ chứa chuỗi JSON dài hơn
  snprintf(payload, sizeof(payload), "{\"name\":\"%s\"}", finalNode.name.c_str());
  client.publish("delivery/location", payload);

  Serial.print("Đã gửi vị trí cuối cùng (name): ");
  Serial.println(finalNode.name.c_str());
}

// ====== A* Algorithm Implementation ======

struct CompareF {
    bool operator()(const pair<int, float>& a, const pair<int, float>& b) {
        return a.second > b.second; // min-heap dựa vào fCost
    }
};

float heuristic(const Node& a, const Node& b) {
    return abs(a.x - b.x) + abs(a.y - b.y); // Manhattan distance
}

void resetGraph(vector<Node>& graph) {
    for (auto& node : graph) node.reset();
}

vector<int> aStar(int startId, int goalId, vector<Node>& graph) {
    resetGraph(graph);

    graph[startId].gCost = 0;
    graph[startId].hCost = heuristic(graph[startId], graph[goalId]);
    graph[startId].fCost = graph[startId].hCost;

    priority_queue<pair<int, float>, vector<pair<int, float>>, CompareF> openSet;
    unordered_map<int, bool> inOpenSet;

    openSet.push({startId, graph[startId].fCost});
    inOpenSet[startId] = true;

    while (!openSet.empty()) {
        int currentId = openSet.top().first;
        openSet.pop();
        inOpenSet[currentId] = false;

        if (currentId == goalId) {
            // reconstruct path (danh sách các id)
            vector<int> path;
            for (int id = goalId; id != -1; id = graph[id].parentId)
                path.push_back(id);
            reverse(path.begin(), path.end());
            return path;
        }

        for (const Edge& edge : graph[currentId].neighbors) {
            int neighborId = edge.neighborId;
            float tentativeG = graph[currentId].gCost + edge.cost;

            if (tentativeG < graph[neighborId].gCost) {
                graph[neighborId].gCost = tentativeG;
                graph[neighborId].hCost = heuristic(graph[neighborId], graph[goalId]);
                graph[neighborId].fCost = graph[neighborId].gCost + graph[neighborId].hCost;
                graph[neighborId].parentId = currentId;

                if (!inOpenSet[neighborId]) {
                    openSet.push({neighborId, graph[neighborId].fCost});
                    inOpenSet[neighborId] = true;
                }
            }
        }
    }

    return {}; // Không tìm thấy đường
}

int findNearestStation(int fromId, vector<Node>& graph) {
    float minCost = numeric_limits<float>::infinity();
    int nearestStationId = -1;

    for (const Node& node : graph) {
        if (node.label == "STATION") {
            vector<int> path = aStar(fromId, node.id, graph);
            if (!path.empty() && graph[node.id].gCost < minCost) {
                minCost = graph[node.id].gCost;
                nearestStationId = node.id;
            }
        }
    }
    return nearestStationId;
}

// ====== Voice Recognition Helper Functions ======
static void audio_inference_callback(uint32_t n_bytes) {
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i];
        if(inference.buf_count >= inference.n_samples) {
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {
    const int32_t i2s_bytes_to_read = (uint32_t)arg;
    size_t bytes_read = i2s_bytes_to_read;

    while (record_status) {
        i2s_read((i2s_port_t)1, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);
        if (bytes_read > 0) {
            for (int x = 0; x < i2s_bytes_to_read/2; x++) {
                sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
            }
            audio_inference_callback(i2s_bytes_to_read);
        }
    }
    vTaskDelete(NULL);
}

static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
    if(!inference.buffer) return false;

    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    if (i2s_init(EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start I2S!");
    }

    ei_sleep(100);
    record_status = true;
    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);
    return true;
}

static bool microphone_inference_record(void) {
    while (inference.buf_ready == 0) delay(10);
    inference.buf_ready = 0;
    return true;
}

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
    return 0;
}

static void microphone_inference_end(void) {
    i2s_deinit();
    free(inference.buffer);
}

static int i2s_init(uint32_t sampling_rate) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
        .sample_rate = sampling_rate,
        .bits_per_sample = (i2s_bits_per_sample_t)16,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = -1,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = 2,
        .ws_io_num = 32,
        .data_out_num = -1,
        .data_in_num = 33,
    };
    esp_err_t ret = i2s_driver_install((i2s_port_t)1, &i2s_config, 0, NULL);
    if (ret == ESP_OK) ret = i2s_set_pin((i2s_port_t)1, &pin_config);
    if (ret == ESP_OK) ret = i2s_zero_dma_buffer((i2s_port_t)1);
    return int(ret);
}

static int i2s_deinit(void) {
    i2s_driver_uninstall((i2s_port_t)1);
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif