package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants.PitcherConstants;
import frc.robot.subsystems.PIDFSubsystem;
import frc.robot.subsystems.TestableSubsystem;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.pid.PIDFTalonSRX;

public class PitcherSS extends SubsystemBase implements TestableSubsystem, PIDFSubsystem {
    private final PIDFTalonSRX motor;

    public PitcherSS() {
        motor = PitcherConstants.MOTOR;
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

    public double getSetpoint() {
        return Conversions.magToDegrees(motor.getSetpoint(), PitcherConstants.GEAR_RATIO);
    }

    /**
     * Sets the intake to the given angle
     *
     * @param setpoint the desired angle in degrees
     */
    @Override
    public void setSetpoint(double setpoint) {
        motor.setIntegralAccumulator(0);
        setpoint = MathUtil.clamp(setpoint, 0, PitcherConstants.MAX_ANGLE);
        setpoint = Conversions.degreesToMag(setpoint, PitcherConstants.GEAR_RATIO);
        motor.setSetpoint(setpoint);
    }

    @Override
    public boolean atSetpoint() {
        return motor.atSetpoint();
    }

    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    @Override
    public double[] getValues() {
        return new double[] {getAngle()};
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("List");

        builder.addDoubleProperty("Stats/Angle", this::getAngle, null);

        builder.addDoubleProperty("Stats/setpoint", () -> Math.round(getSetpoint()), this::setSetpoint);

        builder.addBooleanProperty("Reset Encoder", () -> false, (x) -> {
            if(x)
                resetEncoder();
        });

        SmartDashboard.putData("Pitcher/Motor", motor);
    }
}

