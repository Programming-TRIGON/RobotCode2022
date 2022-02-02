package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ReadableSubsystem extends Subsystem {

    double[] getValues();
}
