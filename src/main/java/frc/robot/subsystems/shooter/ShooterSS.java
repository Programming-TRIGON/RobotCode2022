package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class ShooterSS extends SubsystemBase implements TestableSubsystem, PIDSubsystem {
    private final PIDFTalonSRX masterMotor;

    public ShooterSS() {
        masterMotor = ShooterConstants.LEFT_MOTOR;

        ShooterConstants.LEFT_MOTOR.follow(masterMotor);
        ShooterConstants.RIGHT_MOTOR.follow(masterMotor);
    }

    /**
     * @param setpoint the desired velocity in RPM
     */
    @Override
    public void setSetpoint(double setpoint) {
        masterMotor.setSetpoint(Conversions.RPMToFalcon(setpoint));
    }

    @Override
    public boolean atSetpoint() {
        return masterMotor.atSetpoint();
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
