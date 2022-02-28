package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.PIDCommand;
import frc.robot.constants.RobotConstants.*;
import frc.robot.subsystems.shooter.ShooterCalculations;

public class ShootCG extends ParallelCommandGroup {
    public ShootCG(RobotContainer robotContainer) {
        PIDCommand shooterCMD = new PIDCommand(
                robotContainer.shooterSS,
                () -> ShooterCalculations.calculateVelocity(robotContainer.limelight.getTy()));
        PIDCommand pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS, () -> ShooterCalculations.calculateAngle(robotContainer.limelight.getTy()));

        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                shooterCMD,
                                pitcherCMD
                        ),
                        new GenericTurnToTargetCMD(robotContainer.limelight, robotContainer.swerveSS),
                        new WaitUntilCommand(() ->
                                robotContainer.shooterSS.atSetpoint() && robotContainer.pitcherSS.atSetpoint()
                        ),
                        new MoveMovableSubsystem(robotContainer.transporterSS, () -> TransporterConstants.POWER),
                        new PIDCommand(robotContainer.loaderSS, () -> LoaderConstants.VELOCITY)
                )
        );
    }
}
