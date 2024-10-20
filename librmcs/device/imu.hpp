#pragma once

#include <cmath>

#include <atomic>
#include <numbers>

namespace librmcs::device {

class Imu {
public:
    explicit Imu(
        double sample_freq, double kp, double ki, double q0 = 1, double q1 = 0, double q2 = 0,
        double q3 = 0)
        : inv_sample_freq_(1.0 / sample_freq)
        , double_kp_(2.0 * kp)
        , double_ki_(2.0 * ki)
        , q0_(q0)
        , q1_(q1)
        , q2_(q2)
        , q3_(q3){};

    void store_accelerometer_status(int16_t x, int16_t y, int16_t z) {
        accelerometer_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void store_gyroscope_status(int16_t x, int16_t y, int16_t z) {
        gyroscope_data_.store({x, y, z}, std::memory_order::relaxed);
    }

    void update_status() {
        auto acc = accelerometer_data_.load(std::memory_order::relaxed);
        auto gyro = gyroscope_data_.load(std::memory_order::relaxed);

        auto solve_acc = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };

        gx_ = solve_gyro(gyro.x), gy_ = solve_gyro(gyro.y), gz_ = solve_gyro(gyro.z);
        ax_ = solve_acc(acc.x), ay_ = solve_acc(acc.y), az_ = solve_acc(acc.z);

        mahony_ahrs_update_imu();
    }

    double ax() const { return ax_; }
    double ay() const { return ay_; }
    double az() const { return az_; }

    double gx() const { return gx_; }
    double gy() const { return gy_; }
    double gz() const { return gz_; }

    double q0() const { return q0_; }
    double q1() const { return q1_; }
    double q2() const { return q2_; }
    double q3() const { return q3_; }

private:
    void mahony_ahrs_update_imu() {
        // Madgwick's implementation of Mayhony's AHRS algorithm.
        // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

        double recip_norm;
        double halfvx, halfvy, halfvz;
        double halfex, halfey, halfez;
        double qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
        // normalization)
        if (!((ax_ == 0.0) && (ay_ == 0.0) && (az_ == 0.0))) {

            // Normalize accelerometer measurement
            recip_norm = 1 / std::sqrt(ax_ * ax_ + ay_ * ay_ + az_ * az_);
            ax_ *= recip_norm;
            ay_ *= recip_norm;
            az_ *= recip_norm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q1_ * q3_ - q0_ * q2_;
            halfvy = q0_ * q1_ + q2_ * q3_;
            halfvz = q0_ * q0_ - 0.5 + q3_ * q3_;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = ay_ * halfvz - az_ * halfvy;
            halfey = az_ * halfvx - ax_ * halfvz;
            halfez = ax_ * halfvy - ay_ * halfvx;

            // Compute and apply integral feedback if enabled
            if (double_ki_ > 0.0) {
                // integral error scaled by Ki
                integral_fbx_ += double_ki_ * halfex * (inv_sample_freq_);
                integral_fby_ += double_ki_ * halfey * (inv_sample_freq_);
                integral_fbz_ += double_ki_ * halfez * (inv_sample_freq_);
                // apply integral feedback
                gx_ += integral_fbx_;
                gy_ += integral_fby_;
                gz_ += integral_fbz_;
            } else {
                // prevent integral windup
                integral_fbx_ = 0.0;
                integral_fby_ = 0.0;
                integral_fbz_ = 0.0;
            }

            // Apply proportional feedback
            gx_ += double_kp_ * halfex;
            gy_ += double_kp_ * halfey;
            gz_ += double_kp_ * halfez;
        }

        // Integrate rate of change of quaternion
        gx_ *= (0.5 * (inv_sample_freq_)); // pre-multiply common factors
        gy_ *= (0.5 * (inv_sample_freq_));
        gz_ *= (0.5 * (inv_sample_freq_));
        qa = q0_;
        qb = q1_;
        qc = q2_;
        q0_ += (-qb * gx_ - qc * gy_ - q3_ * gz_);
        q1_ += (qa * gx_ + qc * gz_ - q3_ * gy_);
        q2_ += (qa * gy_ - qb * gz_ + q3_ * gx_);
        q3_ += (qa * gz_ + qb * gy_ - qc * gx_);

        // Normalize quaternion
        recip_norm = 1 / std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
        q0_ *= recip_norm;
        q1_ *= recip_norm;
        q2_ *= recip_norm;
        q3_ *= recip_norm;
    }

    double inv_sample_freq_; // The reciprocal of sampling frequency
    double double_kp_;       // 2 * proportional gain (Kp)
    double double_ki_;       // 2 * integral gain (Ki)

    struct alignas(8) ImuData {
        int16_t x, y, z;
    };
    std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
    static_assert(std::atomic<ImuData>::is_always_lock_free);

    double ax_, ay_, az_, gx_, gy_, gz_;

    // Quaternion of sensor frame relative to auxiliary frame
    double q0_, q1_, q2_, q3_;

    // Integral error terms scaled by Ki
    double integral_fbx_ = 0.0, integral_fby_ = 0.0, integral_fbz_ = 0.0;
};

} // namespace librmcs::device