package frc.robot.commands.commandgroups.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public abstract class AutoCG extends SequentialCommandGroup {
    protected RobotContainer robotContainer;
    protected double startingAngle;

    public AutoCG(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        startingAngle = 0;
    }

    protected void setStartingAngle(double startingAngle) {
        this.startingAngle = startingAngle;
    }

    protected SequentialCommandGroup getStartingCommands() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setStartingAngle(robotContainer.swerveSS.getAngle().getDegrees())),
                new InstantCommand(robotContainer.swerveSS::resetGyro));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotContainer.swerveSS.setAngle(startingAngle + robotContainer.swerveSS.getAngle().getDegrees());
    }
}
