#include <cmath>

namespace filter {
class RCfilter {
 public:
    // f = 1 / 2*pi*RC, f_cutoff = 3*f;
    RCfilter(double frequency, double T) : f_(frequency), T_(T) {}
    double FilterOutput(double now_value) {
       double RC = 1.0 / (2 * M_PI * f_);
       double K = 1.0 / (1.0 + RC / T_);
       return K * now_value + (1.0 - K) * last_value_;
    }
    
 private:
    double f_;
    double T_;
    double last_value_ = 0.0;
};
}   // namespace filter