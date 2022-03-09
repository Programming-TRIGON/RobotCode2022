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

import java.util.function.DoubleSupplier;

public class ShootCG extends ParallelCommandGroup {
    public ShootCG(RobotContainer robotContainer, DoubleSupplier velocity, boolean isManual) {
        PIDCommand shooterCMD = new PIDCommand(
                robotContainer.shooterSS, isManual ? velocity :
                                          () -> ShootingCalculations.calculateVelocity(
                                                  robotContainer.limelight.getDistance()));
        PIDCommand pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS,
                isManual ? () -> 15 : () -> ShootingCalculations.calculateAngle(robotContainer.limelight
                        .getDistance()));

        addCommands(
                shooterCMD,
                pitcherCMD,
                new GenericTurnToTargetCMD(
                        robotContainer.limelight, robotContainer.swerveSS),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() ->
                                robotContainer.shooterSS.atSetpoint() && (robotContainer.limelight.getTv() || isManual)
                        ),
                        new ParallelCommandGroup(
                                new MoveMovableSubsystem(robotContainer.loaderSS, () -> LoaderConstants.POWER),
                                new SequentialCommandGroup(
                                        new WaitCommand(RobotConstants.ShooterConstants.TRANSPORTER_WAIT_TIME),
                                        new MoveMovableSubsystem(
                                                robotContainer.transporterSS, () -> TransporterConstants.POWER)
                                )
                        )
                )
        );
    }
}
