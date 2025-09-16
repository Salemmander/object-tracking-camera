

class PID
{
public:
    PID(double kp, double ki, double kd) noexcept : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

    double compute(double error, double dt) noexcept
    {
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() noexcept
    {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double prev_error_;
};