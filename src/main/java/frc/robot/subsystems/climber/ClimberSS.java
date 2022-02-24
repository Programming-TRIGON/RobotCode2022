package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.pid.PIDFTalonFX;

public class ClimberSS extends OverridableSubsystem {
    private final PIDFTalonFX leftMotor;
    private final PIDFTalonFX rightMotor;

    public ClimberSS() {
        leftMotor = ClimberConstants.LEFT_MOTOR;
        rightMotor = ClimberConstants.RIGHT_MOTOR;
        resetEncoder();
    }

    /**
     * @param Position in ticks
     */
    public void setSetpoint(double Position) {
        leftMotor.setSetpoint(MathUtil.clamp(Position, 0, ClimberConstants.MAX_POSITION));
    }

    public double getLeftPosition() {
        return leftMotor.getSelectedSensorPosition();
    }

    public double getRightPosition() {
        return rightMotor.getSelectedSensorPosition();
    }

    public double getAveragePosition() {
        return (getLeftPosition() + getRightPosition()) / 2;
    }

    public void resetEncoder() {
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void overriddenMove(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
    }
}
