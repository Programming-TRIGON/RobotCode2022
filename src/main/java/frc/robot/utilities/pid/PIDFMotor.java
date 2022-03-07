package frc.robot.utilities.pid;

import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * An interface used for motors that have a configurable PID
 */
public interface PIDFMotor extends PIDFConfigurable {
    void setCoefs(PIDFCoefs pidfCoefs);

    void set(double output);

    double get();

    @Override
    default void initSendable(SendableBuilder builder) {
        PIDFConfigurable.super.initSendable(builder);
        builder.setSmartDashboardType("PIDFMotor");
        builder.setSafeState(() -> set(0));
        builder.addDoubleProperty("output", this::get, output -> set(isTuning() ? output : get()));
    }
}
