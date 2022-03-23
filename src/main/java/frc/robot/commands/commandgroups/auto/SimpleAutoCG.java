package frc.robot.commands.commandgroups.auto;

import frc.robot.RobotContainer;
import frc.robot.commands.commandgroups.IntakeCG;

public class SimpleAutoCG extends AutoCG {

    public SimpleAutoCG(RobotContainer robotContainer) {
        super(robotContainer);
        addCommands(
                getStartingCommands(),
                new IntakeCG(robotContainer).withTimeout(2),
                new AutoShootCG(robotContainer, () -> -0.2, () -> 0.3)
        );
    }
}
