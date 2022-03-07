package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.PIDCommand;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.constants.RobotConstants.TransporterConstants;
import frc.robot.subsystems.shooter.ShootingCalculations;

public class ShootCG extends ParallelCommandGroup {
    public ShootCG(RobotContainer robotContainer) {
        PIDCommand shooterCMD = new PIDCommand(
                robotContainer.shooterSS,
                //TODO: Make sure this updates dynamically
                () -> ShootingCalculations.calculateVelocity(robotContainer.limelight.getDistance()));
        PIDCommand pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS, () -> ShootingCalculations.calculateAngle(robotContainer.limelight
                .getDistance()));

        addCommands(
                shooterCMD,
                pitcherCMD,
                new GenericTurnToTargetCMD(robotContainer.limelight, robotContainer.swerveSS),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                robotContainer.shooterSS.atSetpoint() && robotContainer.pitcherSS.atSetpoint()
                        ),
                        new ParallelCommandGroup(
                                new MoveMovableSubsystem(
                                        robotContainer.transporterSS, () -> TransporterConstants.DEFAULT_POWER),
                                new PIDCommand(robotContainer.loaderSS, () -> LoaderConstants.VELOCITY)
                        )
                )
        );
    }
}
