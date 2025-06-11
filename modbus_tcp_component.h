#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "modbus_tcp"

class ModbusTCPSensor : public PollingComponent, public Sensor {
public:
    ModbusTCPSensor(const char* host, uint16_t port, uint16_t register_address, 
                    const char* byte_order, uint32_t update_interval_ms) 
        : host_(host), port_(port), register_address_(register_address), 
          byte_order_(byte_order), update_interval_ms_(update_interval_ms),
          last_update_time_(0), accuracy_decimals_(2) {}

    void setup() {
        ESP_LOGD(TAG, "Setting up Modbus TCP sensor at address %d...", register_address_);
    }

    void set_accuracy_decimals(int accuracy) {
        accuracy_decimals_ = accuracy;
    }

    void update() {
        int64_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
        if (current_time - last_update_time_ < update_interval_ms_) {
            return;  // Not time to update yet
        }
        last_update_time_ = current_time;

        // Create socket - use the fully qualified function name to avoid ambiguity
        int sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            return;
        }

        // Set socket timeout
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        // Connect to server
        struct sockaddr_in dest_addr;
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port_);
        
        // Convert hostname to address
        struct hostent *he = ::gethostbyname(host_);
        if (he == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed for host %s: %d", host_, h_errno);
            ::close(sock);
            return;
        }
        
        memcpy(&dest_addr.sin_addr.s_addr, he->h_addr, sizeof(dest_addr.sin_addr.s_addr));

        int err = ::connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Failed to connect to Modbus server %s:%d, errno=%d", host_, port_, errno);
            ::close(sock);
            return;
        }

        // Build Modbus request (Read Holding Registers, function code 0x03)
        uint8_t request[] = {
            0x00, 0x01,  // Transaction ID
            0x00, 0x00,  // Protocol ID
            0x00, 0x06,  // Length
            0x01,        // Unit ID
            0x03,        // Function Code (Read Holding Registers)
            (uint8_t)((register_address_ >> 8) & 0xFF),  // Start Address (High Byte)
            (uint8_t)(register_address_ & 0xFF),        // Start Address (Low Byte)
            0x00, 0x02   // Quantity (Read 2 Registers = 32 bits for FP32)
        };

        // Send request
        err = ::send(sock, request, sizeof(request), 0);
        if (err < 0) {
            ESP_LOGE(TAG, "Error sending data: errno %d", errno);
            ::close(sock);
            return;
        }

        // Read response
        uint8_t response[256];
        int response_len = ::recv(sock, response, sizeof(response), 0);
        ::close(sock);  // Close socket now that we're done with it

        if (response_len < 9) {
            ESP_LOGE(TAG, "Invalid response length: %d", response_len);
            return;
        }

        // Debugging: Print raw response data
        ESP_LOGD(TAG, "Raw Response Data:");
        for (int i = 0; i < response_len; i++) {
            ESP_LOGD(TAG, "Byte %d: 0x%02X", i, response[i]);
        }

        if (response[7] != 0x03) {
            ESP_LOGE(TAG, "Unexpected function code: 0x%02X", response[7]);
            return;
        }

        // Decode value based on byte order
        float value = decode_integer(&response[9], byte_order_);
        ESP_LOGD(TAG, "Register %d: %.2f", register_address_, value);
        
        // Store the value
        last_value_ = value;
        
        // Call callback if set
        if (state_callback_) {
            state_callback_(value);
        }
        this->publish_state(value);
    }
    
    // Method to read the current value
    float get_state() const {
        return last_value_;
    }
    
    // Set callback function to be called when value is updated
    void set_state_callback(void (*callback)(float)) {
        state_callback_ = callback;
    }

private:
    const char* host_;
    uint16_t port_;
    uint16_t register_address_;
    const char* byte_order_;
    uint32_t update_interval_ms_;
    int64_t last_update_time_;
    int accuracy_decimals_;
    float last_value_ = 0.0f;
    void (*state_callback_)(float) = nullptr;

    float decode_float(uint8_t *data, const char* byte_order) {
        uint32_t raw_value = 0;

        if (strcmp(byte_order, "AB_CD") == 0) {
            raw_value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
        } else if (strcmp(byte_order, "CD_AB") == 0) {
            raw_value = (data[2] << 24) | (data[3] << 16) | (data[0] << 8) | data[1];
        } else if (strcmp(byte_order, "DC_BA") == 0) {
            raw_value = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        } else {
            ESP_LOGE(TAG, "Invalid byte order: %s", byte_order);
            return 0.0f;
        }

        float value;
        memcpy(&value, &raw_value, sizeof(float));
        return value;
    }

    uint32_t decode_integer(uint8_t *data, const char* byte_order) {
        uint32_t raw_value = 0;
    
        if (strcmp(byte_order, "AB_CD") == 0) {
            raw_value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
        } else if (strcmp(byte_order, "CD_AB") == 0) {
            raw_value = (data[2] << 24) | (data[3] << 16) | (data[0] << 8) | data[1];
        } else if (strcmp(byte_order, "DC_BA") == 0) {
            raw_value = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        } else {
            ESP_LOGE(TAG, "Invalid byte order: %s", byte_order);
            return 0;
        }
    
        return raw_value; // Return integer value
    }
};
