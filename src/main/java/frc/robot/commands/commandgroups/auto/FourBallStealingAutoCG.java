package frc.robot.commands.commandgroups.auto;

import frc.robot.RobotContainer;
import frc.robot.commands.commandgroups.IntakeCG;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

public class FourBallStealingAutoCG extends AutoCG {
    public FourBallStealingAutoCG(RobotContainer robotContainer) {
        super(robotContainer);
        this.robotContainer = robotContainer;
        startingAngle = 0;
        addCommands(
                getStartingCommands(),
                new IntakeCG(robotContainer).withTimeout(1.5),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.0, () -> -0.35, () -> 0.0, false).withTimeout(
                        1.5),
                new AutoShootCG(robotContainer, () -> -0.3).withTimeout(4.5),
                new AutoCollectCG(robotContainer, () -> 0.35).withTimeout(3),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.0, () -> 0.0, () -> 0.35, false).withTimeout(
                        0.5),
                new AutoCollectCG(robotContainer, () -> -0.3).withTimeout(2),
                new AutoShootCG(robotContainer, () -> -0.3));
    }
}
