#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <chrono>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

#include <crow.h>

#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using namespace unitree::common;
using namespace unitree::robot;

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

enum class ExecutionStatus {
    SUCCESS,
    TIMEOUT,
    ERROR
};

using ExecutionResult = std::pair<ExecutionStatus, std::string>;

/* Utils */
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

class PID {
public:
    PID(float kp, float ki, float kd, float rate,
        float cutoff_freq = 0.0f,
        float i_limit = 0.0f,
        float o_limit = 0.0f) {
        init(kp, ki, kd, rate, cutoff_freq, i_limit, o_limit);
    }

    void init(float kp, float ki, float kd, float rate,
              float cutoff_freq = 0.0f,
              float i_limit = 0.0f,
              float o_limit = 0.0f) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->rate = rate;
        this->dt = 1.0f / rate;
        this->i_limit = i_limit;
        this->o_limit = o_limit;
        this->last_error = 0.0f;
        this->integral = 0.0f;
        this->filtered_derivative = 0.0f;
        this->tau = (cutoff_freq > 0.0f) ? (1.0f / (2.0f * M_PI * cutoff_freq)) : 0.0f;
    }

    float update(float error) {
        float output = kp * error;

        float raw_derivative = (error - last_error) * rate;
        last_error = error;

        if (tau > 0.0f) {
            float alpha = dt / (tau + dt);
            filtered_derivative = filtered_derivative * (1.0f - alpha) + raw_derivative * alpha;
        } else {
            filtered_derivative = raw_derivative;
        }

        output += kd * filtered_derivative;

        integral += error * dt;
        if (i_limit > 0.0f) {
            integral = std::clamp(integral, -i_limit, i_limit);
        }

        output += ki * integral;

        if (o_limit > 0.0f) {
            output = std::clamp(output, -o_limit, o_limit);
        }

        return output;
    }

    void reset() {
        last_error = 0.0f;
        integral = 0.0f;
        filtered_derivative = 0.0f;
    }

private:
    float kp;
    float ki;
    float kd;
    float rate;
    float dt;
    float last_error;
    float integral;
    float tau;
    float filtered_derivative;
    float i_limit;
    float o_limit;
};

class Action {
public:
    float _startPos[12]; // initial gesture
    float _endPos[12]; // target gesture
    float duration; // s
    
    // Constructor to copy array values
    Action(const float start[12], const float end[12], float dur) : duration(dur) {
        std::copy(start, start + 12, _startPos);
        std::copy(end, end + 12, _endPos);
    }
};

/* Main class */
class Go2RemoteControl {
public:
    Go2RemoteControl() {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&Go2RemoteControl::_HighStateHandler, this, std::placeholders::_1), 1);

        /*create publisher*/
        lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        lowcmd_publisher->InitChannel();

        /*create subscriber*/
        lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        lowstate_subscriber->InitChannel(std::bind(&Go2RemoteControl::_LowStateHandler, this, std::placeholders::_1), 1);
    };

    void _HighStateHandler(const void *message) {
        high_state = *(unitree_go::msg::dds_::SportModeState_ *)message;

        // std::cout << "Position: " << high_state.position()[0] << ", " << high_state.position()[1] << ", " << high_state.position()[2] << std::endl;
        // std::cout << "IMU rpy: " << high_state.imu_state().rpy()[0] << ", " << high_state.imu_state().rpy()[1] << ", " << high_state.imu_state().rpy()[2] << std::endl;
    };

    void _LowStateHandler(const void *message) {
        low_state = *(unitree_go::msg::dds_::LowState_ *)message;
    };

    void _InitLowCmd() {
        low_cmd.head()[0] = 0xFE;
        low_cmd.head()[1] = 0xEF;
        low_cmd.level_flag() = 0xFF;
        low_cmd.gpio() = 0;

        for(int i=0; i<20; i++)
        {
            low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
            low_cmd.motor_cmd()[i].q() = (PosStopF);
            low_cmd.motor_cmd()[i].kp() = (0);
            low_cmd.motor_cmd()[i].dq() = (VelStopF);
            low_cmd.motor_cmd()[i].kd() = (0);
            low_cmd.motor_cmd()[i].tau() = (0);
        }
    }

    ExecutionResult LookUp(float duration = 4.0) {
        _InitLowCmd();
        float _startPos[12] = {0};
        float _headUp[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
            0.0, 0.67, -1.8, 0.0, 0.67, -1.8};
        
        for (int i = 0; i < 12; i++) {
            _startPos[i] = low_state.motor_state()[i].q();
        }

        Action actions[3] = {
            Action(_startPos, _headUp, 2.0f),
            Action(_headUp, _headUp, 1.0f),
            Action(_headUp, _startPos, 2.0f)
        };

        // Execute each action in the sequence
        for (int i = 0; i < 3; i++) {
            const Action& action = actions[i];
            float duration = action.duration;
            int steps = static_cast<int>(duration / dt); // Number of steps for interpolation

            for (int step = 0; step <= steps; step++) {
                float percent = static_cast<float>(step) / steps;

                for (int j = 0; j < 12; j++) {
                    // Interpolate between the start and end positions
                    low_cmd.motor_cmd()[j].q() = (1 - percent) * action._startPos[j] + percent * action._endPos[j];
                    low_cmd.motor_cmd()[j].dq() = 0;
                    low_cmd.motor_cmd()[j].kp() = 60;
                    low_cmd.motor_cmd()[j].kd() = 5;
                    low_cmd.motor_cmd()[j].tau() = 0;
                }

                // Calculate and set the CRC for the command
                low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);

                // Publish the command
                lowcmd_publisher->Write(low_cmd);

                // Sleep for the control step duration
                usleep(static_cast<useconds_t>(dt * 1000000));
            }
        }

        return {ExecutionStatus::SUCCESS, "LookUp action completed successfully."};
    }

    ExecutionResult stand_down() {
        if (sport_client.StandDown() != 0) {
            std::cerr << "Failed to send stand down command." << std::endl;
            return {ExecutionStatus::ERROR, "Failed to send stand down command."};
        }
        return {ExecutionStatus::SUCCESS, ""};
    }

    ExecutionResult stand_up() {
        if (sport_client.RecoveryStand() != 0) {
            std::cerr << "Failed to send stand up command." << std::endl;
            return {ExecutionStatus::ERROR, "Failed to send stand up command."};
        }
        return {ExecutionStatus::SUCCESS, ""};
    }

    ExecutionResult rotate(double delta_rad, double timeout = 8.0) {
        double initial_yaw = high_state.imu_state().rpy()[2];
        double yaw_target = initial_yaw + delta_rad;
    
        double accumulated_angle = 0.0;
        auto now = std::chrono::system_clock::now();
        auto end_time = now + std::chrono::duration<double>(timeout);
    
        double prev_yaw = initial_yaw;
        while (now < end_time) {
            now = std::chrono::system_clock::now();
    
            double yaw_current = high_state.imu_state().rpy()[2];
            double yaw_diff = yaw_current - prev_yaw;
    
            // Handle yaw wrap-around
            if (yaw_diff > M_PI) {
                yaw_diff -= 2 * M_PI;
            } else if (yaw_diff < -M_PI) {
                yaw_diff += 2 * M_PI;
            }
    
            accumulated_angle += yaw_diff;
            prev_yaw = yaw_current;
    
            // Calculate remaining angle to rotate
            double remaining_angle = delta_rad - accumulated_angle;
    
            if (fabs(remaining_angle) < control_error_yaw) {
                std::cout << "Rotation completed successfully." << std::endl;
                return {ExecutionStatus::SUCCESS, ""};
            }
    
            double vyaw = pid_yaw.update(remaining_angle);
            sport_client.Move(0, 0, vyaw);
    
            usleep(static_cast<useconds_t>(dt * 1000000));
        }
    
        return {ExecutionStatus::TIMEOUT, "Rotation timed out before reaching the target angle."};
    }

    ExecutionResult move(double dx, double dy, bool body_frame = true, double timeout = 1.0) {
        // World frame coordinates
        double initial_x = high_state.position()[0];
        double initial_y = high_state.position()[1];
        double initial_yaw = high_state.imu_state().rpy()[2];
        double yaw_target = initial_yaw;
    
        double target_x;
        double target_y;
    
        if (body_frame) {
            // Convert to body frame coordinates
            double body_dx = dx * cos(initial_yaw) - dy * sin(initial_yaw);
            double body_dy = dx * sin(initial_yaw) + dy * cos(initial_yaw);
            target_x = initial_x + body_dx;
            target_y = initial_y + body_dy;
        } else {
            // Use world frame coordinates
            target_x = initial_x + dx;
            target_y = initial_y + dy;
        }
    
        // printf("Moving to target position: x: %.2f, y: %.2f\n", target_x, target_y);
    
        auto now = std::chrono::system_clock::now();
        auto end_time = now + std::chrono::duration<double>(timeout);
    
        while (now < end_time) {
            now = std::chrono::system_clock::now();
    
            double current_x = high_state.position()[0];
            double current_y = high_state.position()[1];
            double current_yaw = high_state.imu_state().rpy()[2];
    
            double remaining_x = target_x - current_x;
            double remaining_y = target_y - current_y;
    
            if (fabs(remaining_x) < control_error && fabs(remaining_y) < control_error) {
                std::cout << "Movement completed successfully." << std::endl;
                return {ExecutionStatus::SUCCESS, ""};
            }
    
            // Compute world-frame velocities
            double vx = pid_x.update(remaining_x);
            double vy = pid_y.update(remaining_y);
    
            // Transform world-frame velocities to body-frame velocities
            double v_body_x = vx * cos(current_yaw) + vy * sin(current_yaw);
            double v_body_y = -vx * sin(current_yaw) + vy * cos(current_yaw);
    
            // Compute yaw velocity
            double vyaw = pid_yaw.update(yaw_target - current_yaw);
    
            // printf("Current position: x: %.2f, y: %.2f, target position: x: %.2f, y: %.2f\n", current_x, current_y, target_x, target_y);
            // printf("Body frame velocities: vx: %.2f, vy: %.2f, vyaw: %.2f\n", v_body_x, v_body_y, vyaw);
    
            // Send body-frame velocities to the robot
            sport_client.Move(v_body_x, v_body_y, vyaw);
    
            usleep(static_cast<useconds_t>(dt * 1000000));
        }
    
        return {ExecutionStatus::TIMEOUT, "Movement timed out before reaching the target position."};
    }

    unitree_go::msg::dds_::SportModeState_ high_state{}; // default init
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    PID pid_yaw{10.0, 0.0, 0.0, 100.0, 10.0, 0.5, 2.0}; // PID controller for yaw
    PID pid_x{2, 0.0, 0.0, 100.0, 10.0, 0.5, 1.0}; // PID controller for x-axis
    PID pid_y{2, 0.0, 0.0, 100.0, 10.0, 0.5, 1.0}; // PID controller for y-axis

    double ct = 0;         // running time
    int flag = 0;          // flag for special motion
    float dt = 0.01;      // control step: 0.001~0.01
    double control_error_yaw = 0.01; // control error
    double control_error = 0.05; // control error
};

int main(int argc, char **argv) {
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    Go2RemoteControl rc;

    sleep(1); // Wait for 1 second to obtain a stable state
    
    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([](){
        return "Go2 is ready!";
    });

    // Control API route
    CROW_ROUTE(app, "/control").methods(crow::HTTPMethod::POST)([&rc](const crow::request& req) {
        auto body = crow::json::load(req.body);
        if (!body) {
            return crow::response(400, "Invalid JSON");
        }

        std::string command = body["command"].s();
        if (command == "move") {
            double dx = body["dx"].d();
            double dy = body["dy"].d();
            bool body_frame = body["body_frame"].b();
            double timeout = body["timeout"].d();

            auto result = rc.move(dx, dy, body_frame, timeout);
            return crow::response(200, crow::json::wvalue{
                {"status", static_cast<int>(result.first)},
                {"message", result.second}
            });
        } else if (command == "rotate") {
            double delta_rad = body["delta_rad"].d();
            double timeout = body["timeout"].d();

            auto result = rc.rotate(delta_rad, timeout);
            return crow::response(200, crow::json::wvalue{
                {"status", static_cast<int>(result.first)},
                {"message", result.second}
            });
        } else if (command == "stand_up") {
            auto result = rc.stand_up();
            return crow::response(200, crow::json::wvalue{
                {"status", static_cast<int>(result.first)},
                {"message", result.second}
            });
        } else if (command == "stand_down") {
            auto result = rc.stand_down();
            return crow::response(200, crow::json::wvalue{
                {"status", static_cast<int>(result.first)},
                {"message", result.second}
            });
        } else if (command == "look_up") {
            auto result = rc.LookUp();
            return crow::response(200, crow::json::wvalue{
                {"status", static_cast<int>(result.first)},
                {"message", result.second}
            });
        } else {
            return crow::response(400, "Unknown command");
        }
    });

    app.port(18080).multithreaded().run();
    
    return 0;
}