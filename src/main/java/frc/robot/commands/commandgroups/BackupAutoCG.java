package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

import java.util.function.DoubleSupplier;

public class BackupAutoCG extends SequentialCommandGroup {
    public BackupAutoCG(RobotContainer robotContainer, DoubleSupplier velocity, boolean isManual) {
        addCommands(
                new InstantCommand(() -> robotContainer.swerveSS.resetGyro()),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.0, () -> 0.35, () -> 0.0, true).raceWith(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robotContainer.intakeOpenerSS.setState(true)),
                                        new ParallelCommandGroup(
                                                new MoveMovableSubsystem(
                                                        robotContainer.intakeSS,
                                                        () -> RobotConstants.IntakeConstants.POWER),
                                                new MoveMovableSubsystem(
                                                        robotContainer.transporterSS,
                                                        () -> RobotConstants.TransporterConstants.POWER))))
                        .withTimeout(3),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.0, () -> -0.2, () -> 0.3, true).withInterrupt(
                        () -> robotContainer.limelight.getTv()),
                new ShootCG(robotContainer, velocity, isManual),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.2, () -> 0.0, () -> 0.1, true).withInterrupt(
                        () -> robotContainer.limelight.getTv()),

                new ShootCG(robotContainer, velocity, isManual)
        );
    }
}
