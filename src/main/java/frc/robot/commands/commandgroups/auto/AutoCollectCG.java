package frc.robot.commands.commandgroups.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.commandgroups.IntakeCG;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

import java.util.function.Supplier;

public class AutoCollectCG extends SequentialCommandGroup {
    public AutoCollectCG(RobotContainer robotContainer, Supplier<Double> rotPower, Supplier<Double> yPower) {
        addCommands(
                new SupplierDriveCMD(
                        robotContainer.swerveSS, () -> 0.0, yPower, rotPower, true).withInterrupt(
                        () -> robotContainer.cargoLimelight.getTv()),
                new IntakeCG(robotContainer));
    }

    public AutoCollectCG(RobotContainer robotContainer, Supplier<Double> rotPower) {
        this(robotContainer, rotPower, () -> 0.0);
    }
}
