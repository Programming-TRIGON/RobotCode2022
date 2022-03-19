package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

public class BackupAutoCG extends SequentialCommandGroup {
    private RobotContainer robotContainer;
    private double startingAngle;

    public BackupAutoCG(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        startingAngle = 0;
        addCommands(
                new InstantCommand(() -> setStartingAngle(robotContainer.swerveSS.getAngle().getDegrees())),
                new InstantCommand(robotContainer.swerveSS::resetGyro),
                new InstantCommand(() -> robotContainer.intakeOpenerSS.setState(true)),
                new WaitCommand(1),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.0, () -> 0.35, () -> 0.0, false).alongWith(
                                new ParallelCommandGroup(
                                        new MoveMovableSubsystem(
                                                robotContainer.intakeSS,
                                                () -> RobotConstants.IntakeConstants.POWER),
                                        new MoveMovableSubsystem(
                                                robotContainer.transporterSS,
                                                () -> RobotConstants.TransporterConstants.POWER)))
                        .withTimeout(2),
                new ParallelCommandGroup(
                        new MoveMovableSubsystem(
                                robotContainer.intakeSS,
                                () -> RobotConstants.IntakeConstants.POWER),
                        new MoveMovableSubsystem(
                                robotContainer.transporterSS,
                                () -> RobotConstants.TransporterConstants.POWER),
                        new SupplierDriveCMD(
                                robotContainer.swerveSS, () -> 0.0, () -> -0.2, () -> 0.3, true)).withInterrupt(
                        () -> robotContainer.hubLimelight.getTv()),
                new ShootCG(robotContainer)
        );
    }

    public void setStartingAngle(double startingAngle) {
        this.startingAngle = startingAngle;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotContainer.swerveSS.setAngle(startingAngle + robotContainer.swerveSS.getAngle().getDegrees());
    }
}
