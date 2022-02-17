package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.pid.PIDFTalonFX;

public class ClimberSS extends OverridableSubsystem {
    private final PIDFTalonFX rightMotor;
    private final PIDFTalonFX leftMotor;

    public ClimberSS() {
        leftMotor = ClimberConstants.LEFT_MOTOR;
        rightMotor = ClimberConstants.RIGHT_MOTOR;
    }

    public void setDesiredPosition(double desiredPosition) {
        leftMotor.setSetpoint(MathUtil.clamp(desiredPosition, 0, ClimberConstants.MAX_POSITION));
    }

    public double getDesiredPosition() {
        return leftMotor.getSetpoint();
    }

    public double getPosition() {
        return leftMotor.getSelectedSensorPosition();
    }

    @Override
    public void overriddenMove(double power) {
        rightMotor.set(power);
        leftMotor.set(power);
    }
}
