#include <librmcs/client/cboard.hpp>

class MyRobot : public librmcs::client::CBoard {
public:
    explicit MyRobot(int32_t usb_pid = -1)
        : CBoard(usb_pid) {}

private:
    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        LOG_INFO("accelerometer: x = %d, y = %d, z = %d", x, y, z);
    }
};

int main() {
    MyRobot my_robot{};
    my_robot.handle_events();
}