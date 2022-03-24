package frc.robot.commands.commandgroups;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.commands.GenericTurnToTargetCMD;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.PIDCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.constants.RobotConstants.TransporterConstants;
import frc.robot.subsystems.shooter.ShootCMD;

import java.util.function.DoubleSupplier;

public class ShootFromCloseCG extends ParallelCommandGroup {
    GenericTurnToTargetCMD ttt;
    PIDCommand pitcher;
    ShootCMD shooter;
    RobotContainer robotContainer;

    public ShootFromCloseCG(RobotContainer robotContainer, DoubleSupplier velocity, DoubleSupplier angle) {

        ShootCMD shootCMD = new ShootCMD(
                robotContainer.shooterSS,
                velocity);
        PIDCommand pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS,
                angle);

        shooter = shootCMD;
        pitcher = pitcherCMD;
        this.robotContainer = robotContainer;
        addCommands(
                shootCMD,
                new ParallelCommandGroup(
                        pitcherCMD,
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> shootCMD.atSetpoint()),
                                new ParallelCommandGroup(
                                        new MoveMovableSubsystem(robotContainer.loaderSS, () -> LoaderConstants.POWER),
                                        new SequentialCommandGroup(
                                                new ParallelRaceGroup(
                                                        new WaitUntilCommand(() -> shootCMD.getBallsShot() >= 1),
                                                        new WaitCommand(
                                                                RobotConstants.ShooterConstants.TRANSPORTER_WAIT_TIME),
                                                        new WaitCommand(0)
                                                ),
                                                new WaitUntilCommand(shootCMD::atSetpoint),
                                                new MoveMovableSubsystem(
                                                        robotContainer.transporterSS,
                                                        () -> TransporterConstants.POWER).withTimeout(0.1),
                                                new WaitUntilCommand(() -> shootCMD.atSetpoint()),
                                                new MoveMovableSubsystem(
                                                        robotContainer.transporterSS,
                                                        () -> TransporterConstants.POWER))))));
    }

    public ShootFromCloseCG(RobotContainer robotContainer) {
        this(robotContainer, () -> 2500, () -> 1);
    }

    @Override
    public boolean isFinished() {
        return shooter.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robotContainer.transporterSS.stopMoving();
        robotContainer.loaderSS.stopMoving();
        robotContainer.shooterSS.stopMoving();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("ttt", ttt::atSetpoint, null);
        builder.addBooleanProperty("shooter", shooter::atSetpoint, null);
        builder.addDoubleProperty("balls", shooter::getBallsShot, null);
        builder.addBooleanProperty("pitcher", pitcher::atSetpoint, null);
    }
}
