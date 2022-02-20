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
    }

    public void resetEncoder() {
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.getSelectedSensorPosition(0);
    }

    public double getDesiredPosition() {
        return leftMotor.getSetpoint();
    }

    public void setDesiredPosition(double desiredPosition) {
        leftMotor.setSetpoint(MathUtil.clamp(desiredPosition, 0, ClimberConstants.MAX_POSITION));
    }

    public double getPositionLeft() {
        return leftMotor.getSelectedSensorPosition();
    }
    
    public double getPositionRight() {
        return rightMotor.getSelectedSensorPosition();
    }

    public double getAveragePosition() {
        return (getPositionLeft() + getPositionRight()) / 2;
    }

    @Override
    public void overriddenMove(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
    }
}
