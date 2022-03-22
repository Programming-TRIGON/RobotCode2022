package frc.robot.commands.commandgroups.auto;

import frc.robot.RobotContainer;

public class StealingAutoCG extends AutoCG {
    public StealingAutoCG(RobotContainer robotContainer) {
        super(robotContainer);
        this.robotContainer = robotContainer;
        startingAngle = 0;
        //        addCommands(
        //                getStartingCommands(),
        //                new IntakeCG(robotContainer)
        //                , );
    }
}
