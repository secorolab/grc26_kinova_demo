#ifndef STIFFNESS_CONTROLLER_HPP
#define STIFFNESS_CONTROLLER_HPP

class StiffnessController {
public:
    explicit StiffnessController(double stiffness_gain = 1.0)
        : k(stiffness_gain)
    {}

    void set_params(double stiffness_gain)
    {
        k = stiffness_gain;
    }

    double control(double error) const
    {
        return k * error;
    }

public:
    double k;  // Stiffness gain
};

#endif // STIFFNESS_CONTROLLER_HPP