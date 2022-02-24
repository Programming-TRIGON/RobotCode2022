package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class LoaderSS extends SubsystemBase implements TestableSubsystem, PIDSubsystem {
    private final PIDFTalonSRX motor;

    public LoaderSS() {
        motor = LoaderConstants.MOTOR;
    }

    @Override
    public void move(double power) {
        motor.set(power);
    }

    /**
     * @param velocity to be set to the motors in RPM
     */
    @Override
    public void setSetpoint(double velocity) {
        motor.setSetpoint(velocity);
    }

    public double getVelocity() {
        return motor.getSelectedSensorVelocity();
    }

    @Override
    public double[] getValues() {
        return new double[] {getVelocity()};
    }
}

