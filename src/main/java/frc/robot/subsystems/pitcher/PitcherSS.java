package frc.robot.subsystems.pitcher;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.PitcherConstants;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class PitcherSS implements TestableSubsystem, PIDSubsystem {
    private final PIDFTalonSRX motor;

    public PitcherSS() {
        motor = PitcherConstants.MOTOR;

        resetEncoder();

        SmartDashboard.putData("Pitcher/motor", motor);
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
     * @param setpoint the desired angle in degrees
     */
    @Override
    public void setSetpoint(double setpoint) {
        setpoint = Conversions.degreesToMag(
                MathUtil.clamp(setpoint, PitcherConstants.OPEN_ANGLE, PitcherConstants.CLOSED_ANGLE),
                PitcherConstants.GEAR_RATIO);
        motor.set(ControlMode.Position, setpoint);
    }

    public void resetEncoder() {
        motor.setSelectedSensorPosition(PitcherConstants.CLOSED_ANGLE);
    }

    @Override
    public double[] getValues() {
        return new double[] {getAngle()};
    }
}

