package frc.robot.subsystems.loader;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.TrigonTalonSRX;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.subsystems.MovableSubsystem;

public class LoaderSS extends SubsystemBase implements MovableSubsystem {
    private final TrigonTalonSRX motor;

    public LoaderSS() {
        motor = LoaderConstants.MOTOR;

        SmartDashboard.putData("Loader/Motor", motor);
    }

    @Override
    public void move(double power) {
        motor.set(power);
    }
}

