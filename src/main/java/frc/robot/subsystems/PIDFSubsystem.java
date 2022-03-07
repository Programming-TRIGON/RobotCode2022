package frc.robot.subsystems;

public interface PIDFSubsystem extends MovableSubsystem {
    void setSetpoint(double setpoint);

    boolean atSetpoint();
}

