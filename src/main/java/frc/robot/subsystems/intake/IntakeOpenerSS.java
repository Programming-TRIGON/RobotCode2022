package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.EncoderConversions;

public class IntakeOpenerSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;

    public IntakeOpenerSS() {
        motor = IntakeOpenerConstants.MOTOR;
    }

    /**
     * @param power to be applied to the motors
     */
    public void setPower(double power){
        motor.set(power);
    }
    @Override
    public void overriddenMove(double power) {
        setPower(power);
    }

    /**
     * @return the current angle of the intake
     */
    public double getAngle() {
        // Converts position ticks (returned by the encoder) to an angle relative to the ground
        // by timesing by 2048 (ticks in a circle) and dividing by 360 (degrees in  circle)
        return EncoderConversions.MagToDegrees(motor.getSelectedSensorPosition());
    }

    /**
     * Sets the intake to the given angle
     *
     * @param degree desired angle in degrees
     */
    public void moveToAngle(double degree) {
        motor.set(ControlMode.Position, degree);
    }

    /**
     * @return if the intake is currently in the open position
     */
    public boolean isOpen() {
        return getAngle() >= IntakeOpenerConstants.OPENED_ANGLE;
    }

    /**
     * @return if the intake is currently in the closed position
     */
    public boolean isClosed() {
        return getAngle() <= IntakeOpenerConstants.CLOSED_ANGLE;
    }
}