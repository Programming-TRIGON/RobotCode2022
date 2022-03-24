package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.climber.ClimbCMD;
import frc.robot.subsystems.climber.ClimberSS;

import java.util.function.Supplier;

public class ClimbCG extends SequentialCommandGroup {
    public ClimbCG(RobotContainer robotContainer, Supplier<Boolean> isEndgame) {
        ClimberSS climberSS = robotContainer.climberSS;
        addCommands(
                new ClimbCMD(climberSS, RobotConstants.ClimberConstants.MIN_LEFT_POSITION,
                        RobotConstants.ClimberConstants.MIN_RIGHT_POSITION, isEndgame),
                new WaitUntilCommand(() -> climberSS.atSetpoint() || isEndgame.get()),
                new ClimbCMD(
                        climberSS, RobotConstants.ClimberConstants.MAX_LEFT_POSITION,
                        RobotConstants.ClimberConstants.MAX_RIGHT_POSITION, isEndgame),
                new WaitUntilCommand(() -> climberSS.atSetpoint() || isEndgame.get()),
                new ClimbCMD(
                        climberSS, RobotConstants.ClimberConstants.MIN_LEFT_POSITION,
                        RobotConstants.ClimberConstants.MIN_RIGHT_POSITION, isEndgame));
    }
}
