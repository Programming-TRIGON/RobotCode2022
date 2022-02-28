package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class IntakeOpenerSS extends OverridableSubsystem implements PIDSubsystem {
    private final PIDFTalonSRX motor;

    public IntakeOpenerSS() {
        motor = IntakeOpenerConstants.MOTOR;
        resetEncoder();
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
        return Conversions.magToDegrees(motor.getSelectedSensorPosition(), IntakeOpenerConstants.GEAR_RATIO);
    }

    /**
     * Sets the intake to the given angle
     *
     * @param setpoint the desired angle in degrees
     */
    @Override
    public void setSetpoint(double setpoint) {
        setpoint = Conversions.degreesToMag(
                MathUtil.clamp(setpoint, 0, RobotConstants.IntakeOpenerConstants.OPENED_ANGLE),
                RobotConstants.IntakeOpenerConstants.GEAR_RATIO);
        motor.set(ControlMode.Position, setpoint);
    }

    @Override
    public boolean atSetpoint() {
        return motor.atSetpoint();
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
        return getAngle() <= 0;
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

    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }
}