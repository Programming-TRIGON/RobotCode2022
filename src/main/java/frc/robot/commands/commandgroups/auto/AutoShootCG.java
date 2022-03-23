package frc.robot.commands.commandgroups.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.commandgroups.ShootCG;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

import java.util.function.Supplier;

public class AutoShootCG extends SequentialCommandGroup {
    public AutoShootCG(RobotContainer robotContainer, Supplier<Double> rotPower, Supplier<Double> yPower) {
        addCommands(
                new SupplierDriveCMD(
                        robotContainer.swerveSS, () -> 0.0, yPower, rotPower, true).withInterrupt(
                        () -> robotContainer.hubLimelight.getTv()),
                new ShootCG(robotContainer));
    }

    public AutoShootCG(RobotContainer robotContainer, Supplier<Double> rotPower) {
        this(robotContainer, rotPower, () -> 0.0);
    }
}
