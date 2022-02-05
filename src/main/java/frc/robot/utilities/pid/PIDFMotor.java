package frc.robot.utilities.pid;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * An interface used for motors that have a configurable PID
 */
public interface PIDFMotor extends PIDFConfigurable {
    void setCoefs(PIDFCoefs pidfCoefs);

    void setTuning(ControlMode controlMode);

    void set(double output);

    double get();

    default void initSendable(SendableBuilder builder) {
        PIDFConfigurable.super.initSendable(builder);
        builder.setSmartDashboardType("PIDFMotor");
        builder.setSafeState(() -> set(0));
        builder.addDoubleProperty("output", this::get, isTuning() ? this::set : null);
    }
}
