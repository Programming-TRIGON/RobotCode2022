package frc.robot.subsystems.intake;

import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.Intake;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;

    public IntakeSS() {
        motor = Intake.INTAKE_MOTOR;
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }

}
