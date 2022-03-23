package frc.robot.commands.commandgroups.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

public class AutoCycleCG extends SequentialCommandGroup {
    protected RobotContainer robotContainer;
    protected double startingAngle;

    public AutoCycleCG(RobotContainer robotContainer, Supplier<Boolean> startTurningRight, double collectTimeout) {
        this.robotContainer = robotContainer;
        startingAngle = 0;
        addCommands(
                new InstantCommand(() -> setStartingAngle(robotContainer.swerveSS.getAngle().getDegrees())),
                //                new AutoCollectCG(robotContainer).withTimeout(collectTimeout),
                new AutoShootCG(robotContainer, () -> -0.2, () -> 0.3));
    }

    protected void setStartingAngle(double startingAngle) {
        this.startingAngle = startingAngle;
    }
}
