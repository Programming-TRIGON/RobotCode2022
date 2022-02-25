package frc.robot.subsystems.pitcher;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.PitcherConstants;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;

public class PitcherSS implements TestableSubsystem {
    private final TrigonTalonSRX motor;

    public PitcherSS() {
        motor = PitcherConstants.MOTOR;
        resetEncoder();
    }

    /**
     * @param power to be applied to the motors
     */
    @Override
    public void move(double power) {
        motor.set(power);
    }

    /**
     * @return the current angle of the intake
     */
    public double getAngle() {
        return Conversions.magToDegrees(
                motor.getSelectedSensorPosition(),
                PitcherConstants.GEAR_RATIO);
    }

    /**
     * Sets the intake to the given angle
     *
     * @param degree desired angle in degrees
     */
    public void setAngle(double degree) {
        degree = Conversions.degreesToMag(
                MathUtil.clamp(degree, PitcherConstants.OPEN_ANGLE, PitcherConstants.CLOSED_ANGLE),
                PitcherConstants.GEAR_RATIO);
        motor.set(ControlMode.Position, degree);
    }

    public void resetEncoder() {
        motor.setSelectedSensorPosition(PitcherConstants.CLOSED_ANGLE);
    }

    @Override
    public double[] getValues() {
        return new double[] {getAngle()};
    }
}
