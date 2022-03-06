package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.IntakeConstants;
import frc.robot.subsystems.OverridableSubsystem;

public class IntakeSS extends OverridableSubsystem {
    private final TrigonTalonSRX motor;

    public IntakeSS() {
        motor = IntakeConstants.MOTOR;

        SmartDashboard.putData("Intake/motor", motor);
    }

    @Override
    public void overriddenMove(double power) {
        motor.set(power);
    }
}
