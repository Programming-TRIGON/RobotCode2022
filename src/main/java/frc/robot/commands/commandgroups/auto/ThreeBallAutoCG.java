package frc.robot.commands.commandgroups.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

public class ThreeBallAutoCG extends SequentialCommandGroup {
    public ThreeBallAutoCG(RobotContainer robotContainer) {
        addCommands(
                new BackupAutoCG(robotContainer).withTimeout(12),
                new SupplierDriveCMD(
                        robotContainer.swerveSS, () -> 0.0, () -> 0.0, () -> -0.05, false).withTimeout(0.2),
                new AutoCollectCG(robotContainer, () -> -0.1).withTimeout(2.5),
                new AutoShootCG(robotContainer, () -> 0.35));
    }
}
