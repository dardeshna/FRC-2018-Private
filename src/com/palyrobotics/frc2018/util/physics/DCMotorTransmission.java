package com.palyrobotics.frc2018.util.physics;

/**
 * Model of a DC motor rotating a shaft.  All parameters refer to the output (e.g. should already consider gearing
 * and efficiency losses).  The motor is assumed to be symmetric forward/reverse.
 */
public class DCMotorTransmission {
	
    public static final double kEpsilon = 1e-12;

    // TODO add electrical constants?  (e.g. current)

    // All units must be SI!
    protected final double speed_per_volt_;  // rad/s per V (no load)
    protected final double torque_per_volt_;  // N m per V (stall)
    protected final double friction_voltage_;  // V
    protected final double current_per_volt_; // A per V (stall)

    public DCMotorTransmission(final double speed_per_volt,
                               final double torque_per_volt,
                               final double friction_voltage,
                               final double current_per_volt) {
        speed_per_volt_ = speed_per_volt;
        torque_per_volt_ = torque_per_volt;
        friction_voltage_ = friction_voltage;
        current_per_volt_ = current_per_volt;
    }

    public double speed_per_volt() {
        return speed_per_volt_;
    }

    public double torque_per_volt() {
        return torque_per_volt_;
    }

    public double friction_voltage() {
        return friction_voltage_;
    }
    
    public double current_per_volt() {
    	return current_per_volt_;
    }

    public double free_speed_at_voltage(final double voltage) {
        if (voltage > kEpsilon) {
            return Math.max(0.0, voltage - friction_voltage()) * speed_per_volt();
        } else if (voltage < kEpsilon) {
            return Math.min(0.0, voltage + friction_voltage()) * speed_per_volt();
        } else {
            return 0.0;
        }
    }

    public double getTorqueForVoltage(final double output_speed, final double voltage) {
        double effective_voltage = voltage;
        if (output_speed > kEpsilon) {
            // Forward motion, rolling friction.
            effective_voltage -= friction_voltage();
        } else if (output_speed < -kEpsilon) {
            // Reverse motion, rolling friction.
            effective_voltage += friction_voltage();
        } else if (voltage > kEpsilon) {
            // System is static, forward torque.
            effective_voltage = Math.max(0.0, voltage - friction_voltage());
        } else if (voltage < -kEpsilon) {
            // System is static, reverse torque.
            effective_voltage = Math.min(0.0, voltage + friction_voltage());
        } else {
            // System is idle.
            return 0.0;
        }
        return torque_per_volt() * (-output_speed / speed_per_volt() + effective_voltage);
    }
    
    public double getCurrentForVoltage(final double output_speed, final double voltage) {
        return current_per_volt() * (-output_speed / speed_per_volt() + voltage);
    }

    public double getVoltageForTorque(final double output_speed, final double torque) {
        double friction_voltage;
        if (output_speed > kEpsilon) {
            // Forward motion, rolling friction.
            friction_voltage = friction_voltage();
        } else if (output_speed < -kEpsilon) {
            // Reverse motion, rolling friction.
            friction_voltage = -friction_voltage();
        } else if (torque > kEpsilon) {
            // System is static, forward torque.
            friction_voltage = friction_voltage();
        } else if (torque < -kEpsilon) {
            // System is static, reverse torque.
            friction_voltage = -friction_voltage();
        } else {
            // System is idle.
            return 0.0;
        }
        return torque / torque_per_volt() + output_speed / speed_per_volt() + friction_voltage;
    }
    
    public double getCurrentForTorque(final double output_speed, final double torque) {
    	double friction_voltage;
        if (output_speed > kEpsilon) {
            // Forward motion, rolling friction.
            friction_voltage = friction_voltage();
        } else if (output_speed < -kEpsilon) {
            // Reverse motion, rolling friction.
            friction_voltage = -friction_voltage();
        } else if (torque > kEpsilon) {
            // System is static, forward torque.
            friction_voltage = friction_voltage();
        } else if (torque < -kEpsilon) {
            // System is static, reverse torque.
            friction_voltage = -friction_voltage();
        } else {
            // System is idle.
            return 0.0;
        }
        
        return torque / torque_per_volt() + friction_voltage * current_per_volt();
    }
}