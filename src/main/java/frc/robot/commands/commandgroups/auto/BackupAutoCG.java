package frc.robot.commands.commandgroups.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.commandgroups.ShootCG;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.SupplierDriveCMD;
import frc.robot.utilities.DriverStationLogger;

public class BackupAutoCG extends SequentialCommandGroup {
    public BackupAutoCG(RobotContainer robotContainer) {
        addCommands(
                new InstantCommand(robotContainer.swerveSS::resetGyro),
                //                 it closes then opens because mechanics
                new InstantCommand(() -> robotContainer.intakeOpenerSS.setState(false)),
                new WaitCommand(0.5),
                new InstantCommand(() -> robotContainer.intakeOpenerSS.setState(true)),
                new WaitCommand(1),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.0, () -> 0.35, () -> 0.0,
                        false).alongWith(
                                new WaitCommand(1).andThen(
                                        new ParallelCommandGroup(
                                                new MoveMovableSubsystem(
                                                        robotContainer.intakeSS,
                                                        () -> RobotConstants.IntakeConstants.POWER),
                                                new MoveMovableSubsystem(
                                                        robotContainer.transporterSS,
                                                        () -> RobotConstants.TransporterConstants
                                                                .POWER))))
                        .withTimeout(1.7),
                new ParallelCommandGroup(
                        new MoveMovableSubsystem(
                                robotContainer.intakeSS,
                                () -> RobotConstants.IntakeConstants.POWER),
                        new MoveMovableSubsystem(
                                robotContainer.transporterSS,
                                () -> RobotConstants.TransporterConstants.POWER),
                        new SupplierDriveCMD(
                                robotContainer.swerveSS, () -> 0.0, () -> 0.0, () -> 0.2, false)).withInterrupt(
                        () -> robotContainer.hubLimelight.getTv()),
                // if you remove this everything just breaks
                new WaitCommand(0.2),
                new InstantCommand(() -> DriverStationLogger.logToDS(
                        "!!!!!!!!!!!!!!!!!!! TV in auto: " + robotContainer.hubLimelight.getTv())),
                new ShootCG(robotContainer),
                new InstantCommand(() -> DriverStationLogger.logToDS(
                        "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!IT STOPPED SHOOTING!!!!!!!!!!!!!!!!!!!!!")),
                new SupplierDriveCMD(
                        robotContainer.swerveSS, () -> 0.0, () -> 0.0, () -> 0.1, false).withInterrupt(
                        () -> robotContainer.hubLimelight.getTv()),
                new ShootCG(robotContainer)
        );
    }
}