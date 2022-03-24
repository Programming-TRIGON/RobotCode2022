package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants.ClimberConstants;
import frc.robot.subsystems.OverridableSubsystem;
import frc.robot.utilities.pid.PIDFTalonFX;

public class ClimberSS extends OverridableSubsystem {
    private final PIDFTalonFX leftMotor;
    private final PIDFTalonFX rightMotor;

    public ClimberSS() {
        leftMotor = ClimberConstants.LEFT_MOTOR;
        rightMotor = ClimberConstants.RIGHT_MOTOR;

        leftMotor.configMaxIntegralAccumulator(0, 430000);
        leftMotor.configClosedLoopPeakOutput(0, 0.8);
        rightMotor.configMaxIntegralAccumulator(0, 430000);
        rightMotor.configClosedLoopPeakOutput(0, 0.8);

        resetEncoders();

        SmartDashboard.putData("Climber/left motor", leftMotor);
        SmartDashboard.putData("Climber/right motor", rightMotor);
    }

    /**
     * @param setpoint desired position in ticks
     */
    public void setSetpoint(double... setpoint) {
        setpoint[0] = MathUtil.clamp(setpoint[0], 0, ClimberConstants.MAX_LEFT_POSITION);
        setpoint[1] = MathUtil.clamp(setpoint[1], 0, ClimberConstants.MAX_RIGHT_POSITION);
        leftMotor.setSetpoint(setpoint[0]);
        rightMotor.setSetpoint(setpoint[1]);
    }

    public boolean atSetpoint() {
        return leftMotor.atSetpoint() && rightMotor.atSetpoint();
    }

    public double getLeftPower() {
        return leftMotor.get();
    }

    public double getRightPower() {
        return rightMotor.get();
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

    public void moveRight(double power) {
        //        if(getRightPosition() >= ClimberConstants.MAX__RIGHT_POSITION
        //                - ClimberConstants.POSITION_TOLERANCE ||
        //                getRightPosition() <= -ClimberConstants.MAX__RIGHT_POSITION
        //                        + ClimberConstants.POSITION_TOLERANCE)
        //            power = 0;
        rightMotor.set(power);
    }

    public void moveLeft(double power) {
        //        if(getLeftPosition() >= ClimberConstants.MAX_LEFT_POSE
        //                - ClimberConstants.POSITION_TOLERANCE ||
        //                getLeftPosition() <= -ClimberConstants.MAX_LEFT_POSE
        //                        + ClimberConstants.POSITION_TOLERANCE)
        //            power = 0;
        leftMotor.set(power);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("List");
        builder.addBooleanProperty("Reset Encoders", () -> false, (x) -> resetEncoders());
        builder.addDoubleProperty("left setpoint", leftMotor::getSetpoint, leftMotor::setSetpoint);
        builder.addDoubleProperty("right setpoint", rightMotor::getSetpoint, rightMotor::setSetpoint);
    }
}
