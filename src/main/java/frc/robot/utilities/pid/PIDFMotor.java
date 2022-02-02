package frc.robot.utilities.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * An interface used for motors that have a configurable PID
 */
public interface PIDFMotor extends PIDFConfigurable {
    void setCoefs(PIDFCoefs pidfCoefs);

    void tunePIDF(ControlMode controlMode);
}
