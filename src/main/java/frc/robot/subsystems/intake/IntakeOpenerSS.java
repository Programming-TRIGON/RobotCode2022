package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.EncoderConversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class IntakeOpenerSS extends OverridableSubsystem {
    private final PIDFTalonSRX motor;

    public IntakeOpenerSS() {
        motor = IntakeOpenerConstants.MOTOR;
        resetEncoder();
    }

    public void resetEncoder(){
        motor.setSelectedSensorPosition(0);
    }
    /**
     * @param power to be applied to the motors
     */

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    /**
     * @return the current angle of the intake
     */
    public double getAngle() {

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

    /**
     * @return the current being given to the motor
     */
    public double getStatorCurrent() {
        return motor.getStatorCurrent();
    }

    /**
     * @return if the motor is currently stalled
     */
    public boolean isStalled() {
        return motor.getStatorCurrent() >= RobotConstants.IntakeOpenerConstants.STALL_CURRENT_LIMIT;
    }

}