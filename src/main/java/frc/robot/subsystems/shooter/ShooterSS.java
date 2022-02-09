package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonFX;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.TestableSubsystem;

public class ShooterSS extends SubsystemBase implements TestableSubsystem {
    private final TrigonTalonFX masterMotor;

    public ShooterSS() {
        masterMotor = ShooterConstants.RIGHT_MOTOR;
        ShooterConstants.RIGHT_MOTOR.follow(masterMotor);
        ShooterConstants.LEFT_MOTOR.follow(masterMotor);
    }

    /**
     * @param voltage to be set to the motors
     */
    public void move(double voltage) {
        masterMotor.setVoltage(voltage);
    }

    /**
     * @return the velocity of the motors in RPM
     */
    public double getVelocityRPM() {
        // timesed by 600 to convert to minutes and divided by 2048 to convert to revolutions
        return masterMotor.getSelectedSensorVelocity() * 600 / 2048;
    }

    public void setRampRate(double rampRate) {
        ShooterConstants.RIGHT_MOTOR.configClosedloopRamp(rampRate);
        ShooterConstants.LEFT_MOTOR.configClosedloopRamp(rampRate);
        ShooterConstants.RIGHT_MOTOR.configOpenloopRamp(rampRate);
        ShooterConstants.LEFT_MOTOR.configOpenloopRamp(rampRate);
    }

    /**
     * @return an array of the current encoder position
     */
    public double[] getValues() {
        return new double[] {masterMotor.getSelectedSensorPosition()};
    }
}

