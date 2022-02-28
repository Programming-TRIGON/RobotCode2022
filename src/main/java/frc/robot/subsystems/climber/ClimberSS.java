package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.subsystems.PIDSubsystem;
import frc.robot.utilities.pid.PIDFTalonFX;

public class ClimberSS extends OverridableSubsystem implements PIDSubsystem {
    private final PIDFTalonFX leftMotor;
    private final PIDFTalonFX rightMotor;

    public ClimberSS() {
        leftMotor = ClimberConstants.LEFT_MOTOR;
        rightMotor = ClimberConstants.RIGHT_MOTOR;

        resetEncoders();
    }

    /**
     * @param setpoint desired position in ticks
     */
    @Override
    public void setSetpoint(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, 0, ClimberConstants.MAX_POSITION);

        leftMotor.setSetpoint(setpoint);
        rightMotor.setSetpoint(setpoint);
    }

    @Override
    public boolean atSetpoint() {
        return leftMotor.atSetpoint() && rightMotor.atSetpoint();
    }

    /**
     * @return left position in ticks
     */
    public double getLeftPosition() {
        return leftMotor.getSelectedSensorPosition();
    }

    /**
     * @return right position in ticks
     */
    public double getRightPosition() {
        return rightMotor.getSelectedSensorPosition();
    }

    /**
     * @return average position in ticks
     */
    public double getAveragePosition() {
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    public void resetEncoders() {
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void overriddenMove(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
    }
}
