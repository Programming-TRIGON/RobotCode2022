package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
        if(isOpening) {
            motor.set(IntakeOpenerConstants.OPENING_POWER);
            motor.setNeutralMode(NeutralMode.Coast);
        } else {
            motor.set(-IntakeOpenerConstants.CLOSING_POWER);
            motor.setNeutralMode(NeutralMode.Brake);
        }
    }
}
