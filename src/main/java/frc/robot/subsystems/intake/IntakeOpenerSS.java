package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeOpener;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeOpenerSS extends OverridableSubsystem {
    private TrigonTalonSRX motor;
    public DigitalInput digitalInput;

    public IntakeOpenerSS() {
        motor = IntakeOpener.INTAKE_OPEN_MOTOR;
        digitalInput = IntakeOpener.INTAKE_OPENER_SWITCH;
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

    public boolean getIsOpen(){
        return digitalInput.get();
    }

    public double getPosition(){
        return motor.getActiveTrajectoryPosition();
    }

}