package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.PIDCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.constants.RobotConstants.TransporterConstants;
import frc.robot.subsystems.shooter.ShootingCalculations;

public class ShootCG extends ParallelCommandGroup {
    public ShootCG(RobotContainer robotContainer) {
        PIDCommand shooterCMD = new PIDCommand(
                robotContainer.shooterSS,
                () -> ShootingCalculations.calculateVelocity(robotContainer.limelight.getDistance()));
        PIDCommand pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS, () -> ShootingCalculations.calculateAngle(robotContainer.limelight
                .getDistance()));
        GenericTurnToTargetCMD turnToTargetCMD = new GenericTurnToTargetCMD(
                robotContainer.limelight, robotContainer.swerveSS);

        addCommands(
                shooterCMD,
                pitcherCMD,
                turnToTargetCMD,
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                robotContainer.shooterSS.atSetpoint() &&
                                        robotContainer.pitcherSS.atSetpoint() &&
                                        turnToTargetCMD.atSetpoint()

                        ),
                        new ParallelCommandGroup(
                                new MoveMovableSubsystem(
                                        robotContainer.transporterSS, () -> TransporterConstants.DEFAULT_POWER),
                                new SequentialCommandGroup(
                                        new WaitCommand(RobotConstants.ShooterConstants.TRANSPORTER_WAIT_TIME),
                                        new MoveMovableSubsystem(robotContainer.loaderSS, () -> LoaderConstants.POWER)
                                )
                        )
                )
        );
    }
}
