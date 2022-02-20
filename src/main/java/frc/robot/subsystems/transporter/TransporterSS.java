package frc.robot.subsystems.transporter;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.TransporterConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class TransporterSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;

    public TransporterSS() {
        motor = TransporterConstants.MOTOR;
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
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
        return motor.getStatorCurrent() > TransporterConstants.STALL_CURRENT_LIMIT;
    }
}

