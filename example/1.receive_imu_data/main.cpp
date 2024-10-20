#include <librmcs/forwarder/cboard.hpp>

class MyRobot : public librmcs::forwarder::CBoard {
public:
    explicit MyRobot(uint16_t usb_pid)
        : CBoard(usb_pid) {}

private:
    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        LOG_INFO("accelerometer: x = %d, y = %d, z = %d", x, y, z);
    }
};

int main() {
    MyRobot my_robot{0x1234};
    my_robot.handle_events();
}