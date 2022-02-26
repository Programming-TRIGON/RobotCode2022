package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class ShooterSS extends SubsystemBase implements TestableSubsystem {
    private final PIDFTalonSRX masterMotor;

    public ShooterSS() {
        masterMotor = ShooterConstants.LEFT_MOTOR;
        ShooterConstants.RIGHT_MOTOR.setInverted(!masterMotor.getInverted());

        ShooterConstants.LEFT_MOTOR.follow(masterMotor);
        ShooterConstants.RIGHT_MOTOR.follow(masterMotor);
    }

    /**
     * @param velocityRPM to be set to the motors
     */
    public void setVelocityRPM(double velocityRPM) {
        masterMotor.setSetpoint(Conversions.RPMToFalcon(velocityRPM));
    }

    /**
     * @param power to be set to the motors
     */
    @Override
    public void move(double power) {
        masterMotor.set(power);
    }

    /**
     * @return the velocity of the motors in RPM
     */
    public double getVelocityRPM() {
        return Conversions.falconToRPM(masterMotor.getSelectedSensorVelocity());
    }

    /**
     * @return an array of the current encoder position
     */
    public double[] getValues() {
        return new double[] {masterMotor.getSelectedSensorPosition()};
    }
}

