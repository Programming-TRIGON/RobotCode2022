package frc.robot.subsystems.Pitcher;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.PitcherConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.EncoderConversions;

public class PitcherSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;

    public PitcherSS() {
        motor = PitcherConstants.MOTOR;
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
        return EncoderConversions.magToDegrees(
                motor.getSelectedSensorPosition(),
                PitcherConstants.GEAR_RATIO);
    }

    /**
     * Sets the intake to the given angle
     *
     * @param degree desired angle in degrees
     */
    public void setPosition(double degree) {
        motor.set(ControlMode.Position, EncoderConversions.degreesToMag(
                MathUtil.clamp(degree, PitcherConstants.CLOSED_ANGLE, PitcherConstants.OPEN_ANGLE),
                PitcherConstants.GEAR_RATIO));
    }

    public void resetEncoder() {
        motor.setSelectedSensorPosition(PitcherConstants.CLOSED_ANGLE);
    }
}

