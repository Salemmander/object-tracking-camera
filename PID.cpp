#include <stdexcept>

class PID
{
public:
    PID(double kp, double ki, double kd) noexcept
        : kp_(kp < 0 ? 0.0 : kp),
          ki_(ki < 0 ? 0.0 : ki),
          kd_(kd < 0 ? 0.0 : kd),
          integral_(0.0),
          prev_error_(0.0) {}

    double compute(double error, double dt) noexcept
    {
        if (dt <= 0)
        {
            return 0.0;
        }
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

extern "C"
{
    PID *PID_new(double kp, double ki, double kd) noexcept
    {
        return new PID(kp, ki, kd);
    }

    void PID_delete(PID *pid) noexcept
    {
        delete pid;
    }

    double PID_compute(PID *pid, double error, double dt) noexcept
    {
        if (!pid)
            return 0.0;
        return pid->compute(error, dt);
    }

    void PID_reset(PID *pid) noexcept
    {
        if (pid)
            pid->reset();
    }
}