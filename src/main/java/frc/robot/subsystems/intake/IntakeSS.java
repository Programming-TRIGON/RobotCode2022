package frc.robot.subsystems.intake;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;

    public IntakeSS() {
        motor = IntakeConstants.MOTOR;
    }

    public void setPower(double power){
        motor.set(power);
    }
    @Override
    public void overriddenMove(double power) {
        setPower(power);
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
        return motor.getStatorCurrent() > RobotConstants.IntakeOpenerConstants.STALL_CURRENT_LIMIT;
    }

}