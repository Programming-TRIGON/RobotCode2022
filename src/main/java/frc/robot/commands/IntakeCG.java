package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.constants.RobotConstants;

public class IntakeCG extends ParallelCommandGroup {
    RobotContainer robotContainer;

    public IntakeCG(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        GoToTargetCMD goToTargetCMD = new GoToTargetCMD(
                robotContainer.swerveSS, robotContainer.cargoLimelight,
                RobotConstants.IntakeConstants.ROTATION_COEFS, () -> 2, 0.5);
        addCommands(
                new InstantCommand(() -> robotContainer.intakeOpenerSS.setState(true)),
                new MoveMovableSubsystem(robotContainer.intakeSS, () -> RobotConstants.IntakeConstants.POWER),
                new MoveMovableSubsystem(robotContainer.transporterSS, () -> RobotConstants.TransporterConstants.POWER),
                goToTargetCMD
        );
        goToTargetCMD.putOnShuffleboard("IntakeCG/GTT");
    }
}