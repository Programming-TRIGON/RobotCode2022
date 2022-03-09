package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

public class BackupAutoCG extends SequentialCommandGroup {
    public BackupAutoCG(RobotContainer robotContainer) {
        GenericTurnToTargetCMD turnToTargetCMD = new GenericTurnToTargetCMD(
                robotContainer.limelight, robotContainer.swerveSS);
        addCommands(
                new SupplierDriveCMD(robotContainer.swerveSS, () -> -0.5, () -> 0.0, () -> 0.0, true).withTimeout(3),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.1, () -> 0.0, () -> 0.3, true).withInterrupt(
                        () -> robotContainer.limelight.getTv()),
                new ShootCG(robotContainer).deadlineWith(turnToTargetCMD),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.2, () -> 0.0, () -> 0.1, true).withInterrupt(
                        () -> robotContainer.limelight.getTv()),
                new ShootCG(robotContainer)
        );
    }
}
