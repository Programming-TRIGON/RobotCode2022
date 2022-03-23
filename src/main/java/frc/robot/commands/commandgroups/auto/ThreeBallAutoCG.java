package frc.robot.commands.commandgroups.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

public class ThreeBallAutoCG extends SequentialCommandGroup {
    public ThreeBallAutoCG(RobotContainer robotContainer) {
        addCommands(
                new SimpleAutoCG(robotContainer),
                new SupplierDriveCMD(
                        robotContainer.swerveSS, () -> 0.0, () -> 0.0, () -> -0.35, true).withTimeout(1),
                new AutoCollectCG(robotContainer, () -> -0.35).withTimeout(3),
                new AutoShootCG(robotContainer, () -> 0.3));
    }
}
