#include <bit>

#include <librmcs/client/cboard.hpp>
#include <librmcs/utility/endian_promise.hpp>

class MyRobot : public librmcs::client::CBoard {
public:
    explicit MyRobot(int32_t usb_pid = -1)
        : CBoard(usb_pid) {}

private:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        (void)is_extended_can_id;
        (void)is_remote_transmission;
        (void)can_data_length;

        if (can_id == 0x201) {
            using librmcs::utility::be_int16_t;

            struct {
                be_int16_t angle;
                be_int16_t velocity;
                be_int16_t current;
                uint8_t temperature;
                uint8_t unused;
            } feedback = std::bit_cast<decltype(feedback)>(can_data);

            LOG_INFO("angle=%d, velocity=%d", (int)feedback.angle, (int)feedback.velocity);
        } else {
            LOG_INFO("unhandled CAN1 device: 0x%x", can_id);
        }
    }
};

int main() {
    MyRobot my_robot{};
    my_robot.handle_events();
}