#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <chrono>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

#ifndef PI
#define PI 3.14159265358979323846f
#endif

class PID {
public:
    PID(float kp, float ki, float kd, float rate,
        float cutoff_freq = 0.0f,
        float i_limit = 0.0f,
        float o_limit = 0.0f)
    {
        init(kp, ki, kd, rate, cutoff_freq, i_limit, o_limit);
    }

    void init(float kp, float ki, float kd, float rate,
              float cutoff_freq = 0.0f,
              float i_limit = 0.0f,
              float o_limit = 0.0f)
    {
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
        this->tau = (cutoff_freq > 0.0f) ? (1.0f / (2.0f * PI * cutoff_freq)) : 0.0f;
    }

    float update(float error)
    {
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

    void reset()
    {
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


enum test_mode {
    /*---Basic motion---*/
    normal_stand,
    balance_stand,
    velocity_move,
    trajectory_follow,
    stand_down,
    stand_up,
    damp,
    recovery_stand,
    /*---Special motion ---*/
    sit, // 8
    rise_sit,
    stretch, // 10
    wallow,
    pose, // 12
    scrape, // 13
    front_flip, // 14
    front_jump,
    front_pounce,
    stop_move = 99
};

class Custom {
public:
    Custom() {
        sport_client.SetTimeout(10.0f);
        sport_client.Init();

        suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
        suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
    };

    // Get initial position
    void GetInitState() {
        px0 = state.position()[0];
        py0 = state.position()[1];
        yaw0 = state.imu_state().rpy()[2];
        std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
    };

    void HighStateHandler(const void *message) {
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;

        // std::cout << "Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
        // std::cout << "IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
    };

    void rotate(double delta_angle, double timeout = 8.0) {
        double initial_yaw = state.imu_state().rpy()[2];
        double yaw_target = initial_yaw + delta_angle * PI / 180.0;
    
        double accumulated_angle = 0.0;
        auto now = std::chrono::system_clock::now();
        auto end_time = now + std::chrono::duration<double>(timeout);
    
        double prev_yaw = initial_yaw;
        while (now < end_time) {
            now = std::chrono::system_clock::now();
    
            double yaw_current = state.imu_state().rpy()[2];
            double yaw_diff = yaw_current - prev_yaw;
    
            // Handle yaw wrap-around
            if (yaw_diff > PI) {
                yaw_diff -= 2 * PI;
            } else if (yaw_diff < -PI) {
                yaw_diff += 2 * PI;
            }
    
            accumulated_angle += yaw_diff;
            prev_yaw = yaw_current;
    
            // Calculate remaining angle to rotate
            double remaining_angle = delta_angle * PI / 180.0 - accumulated_angle;
    
            if (fabs(remaining_angle) < control_error_yaw) {
                break; // Stop rotating when the error is small enough
            }
    
            double vyaw = pid_yaw.update(remaining_angle);
            sport_client.Move(0, 0, vyaw);
    
            usleep(static_cast<useconds_t>(dt * 1000000));
        }
    
        // Optional: Log if the rotation timed out
        if (now >= end_time) {
            std::cerr << "Rotation timed out before reaching the target angle." << std::endl;
        }
    }

    void move(double dx, double dy, bool body_frame = true, double timeout = 1.0) {
        // World frame coordinates
        double initial_x = state.position()[0];
        double initial_y = state.position()[1];
        double initial_yaw = state.imu_state().rpy()[2];
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
    
        printf("Moving to target position: x: %.2f, y: %.2f\n", target_x, target_y);
    
        auto now = std::chrono::system_clock::now();
        auto end_time = now + std::chrono::duration<double>(timeout);
    
        while (now < end_time) {
            now = std::chrono::system_clock::now();
    
            double current_x = state.position()[0];
            double current_y = state.position()[1];
            double current_yaw = state.imu_state().rpy()[2];
    
            double remaining_x = target_x - current_x;
            double remaining_y = target_y - current_y;
    
            if (fabs(remaining_x) < control_error && fabs(remaining_y) < control_error) {
                break; // Stop moving when the error is small enough
            }
    
            // Compute world-frame velocities
            double vx = pid_x.update(remaining_x);
            double vy = pid_y.update(remaining_y);
    
            // Transform world-frame velocities to body-frame velocities
            double v_body_x = vx * cos(current_yaw) + vy * sin(current_yaw);
            double v_body_y = -vx * sin(current_yaw) + vy * cos(current_yaw);
    
            // Compute yaw velocity
            double vyaw = pid_yaw.update(yaw_target - current_yaw);
    
            printf("Current position: x: %.2f, y: %.2f, target position: x: %.2f, y: %.2f\n", current_x, current_y, target_x, target_y);
            printf("Body frame velocities: vx: %.2f, vy: %.2f, vyaw: %.2f\n", v_body_x, v_body_y, vyaw);
    
            // Send body-frame velocities to the robot
            sport_client.Move(v_body_x, v_body_y, vyaw);
    
            usleep(static_cast<useconds_t>(dt * 1000000));
        }
    
        // Optional: Log if the movement timed out
        if (now >= end_time) {
            std::cerr << "Movement timed out before reaching the target position." << std::endl;
        }
    }

    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

    PID pid_yaw{10.0, 0.0, 0.0, 100.0, 10.0, 0.5, 2.0}; // PID controller for yaw
    PID pid_x{2, 0.0, 0.0, 100.0, 10.0, 0.5, 1.0}; // PID controller for x-axis
    PID pid_y{2, 0.0, 0.0, 100.0, 10.0, 0.5, 1.0}; // PID controller for y-axis

    int currentMode = test_mode::stand_down;

    double px0, py0, yaw0; // init position and orientation
    double ct = 0;         // running time
    int flag = 0;          // flag for special motion
    float dt = 0.01;      // control step: 0.001~0.01
    double control_error_yaw = 0.01; // control error
    double control_error = 0.05; // control error
};

int main(int argc, char **argv) {
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");
    Custom custom;

    sleep(1); // Wait for 1 second to obtain a stable state

    custom.GetInitState(); // Get initial position
    
    // while (true) { sleep(1); } // Keep the program running
    double deg = argv[1] ? atof(argv[1]) : 0.0;
    if (deg > 5 || deg < -5) {
        custom.rotate(deg); // Rotate 90 degrees
        return 0;
    }


    double dx = argv[1] ? atof(argv[1]) : 0.0;
    double dy = argv[2] ? atof(argv[2]) : 0.0;
    custom.move(dx, dy); // Rotate 90 degrees
    
    return 0;
}