package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeOpenerConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeOpenerSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;

    public IntakeOpenerSS() {
        motor = IntakeOpenerConstants.MOTOR;

        SmartDashboard.putData("IntakeOpener/motor", motor);
    }

    /**
     * @param power to be applied to the motors
     */
    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    public void toggleState() {
        setState(motor.get() < 0);
    }

    public void setState(boolean isOpening) {
        motor.set(isOpening ? IntakeOpenerConstants.OPENING_POWER : -IntakeOpenerConstants.CLOSING_POWER);
    }
}