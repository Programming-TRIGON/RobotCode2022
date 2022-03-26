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
import frc.robot.constants.RobotConstants.VisionConstants;
import frc.robot.subsystems.shooter.ShootCMD;
import frc.robot.subsystems.shooter.ShootingCalculations;

public class ShootCG extends ParallelCommandGroup {
    GenericTurnToTargetCMD ttt;
    PIDCommand pitcher;
    ShootCMD shooter;

    public ShootCG(RobotContainer robotContainer) {
        GenericTurnToTargetCMD turnToTargetCMD = new GenericTurnToTargetCMD(
                robotContainer.swerveSS,
                () -> -robotContainer.hubLimelight.getTx(),
                robotContainer.hubLimelight::getTv,
                () -> -1.25,
                VisionConstants.HUB_TTT_COEFS,
                1);
        ShootCMD shootCMD = new ShootCMD(
                robotContainer.shooterSS,
                () -> ShootingCalculations.calculateVelocity(robotContainer.hubLimelight.getTy()));
        PIDCommand pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS,
                () -> ShootingCalculations.calculateAngle(robotContainer.hubLimelight.getTy()));

        ttt = turnToTargetCMD;
        shooter = shootCMD;
        pitcher = pitcherCMD;

        turnToTargetCMD.putOnDashboard("ShootCG/TTTCMD");

        addCommands(
                shootCMD,
                new ParallelCommandGroup(
                        pitcherCMD,
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        turnToTargetCMD,
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.5),
                                                new WaitUntilCommand(
                                                        () -> shootCMD.atSetpoint() && turnToTargetCMD.atSetpoint())
                                        )),
                                new ParallelCommandGroup(
                                        new MoveMovableSubsystem(
                                                robotContainer.loaderSS, () -> LoaderConstants.POWER),
                                        new SequentialCommandGroup(
                                                new ParallelRaceGroup(
                                                        new WaitUntilCommand(
                                                                () -> shootCMD.getBallsShot() >= 1),
                                                        new WaitCommand(
                                                                RobotConstants.ShooterConstants.TRANSPORTER_WAIT_TIME),
                                                        // this is dumb but keep it for now
                                                        new WaitCommand(0)
                                                ),
                                                new WaitUntilCommand(shootCMD::atSetpoint),
                                                new MoveMovableSubsystem(
                                                        robotContainer.transporterSS,
                                                        () -> TransporterConstants.POWER).withTimeout(0.1),
                                                new WaitCommand(0.2),
                                                new WaitUntilCommand(() -> shootCMD.atSetpoint()),
                                                new MoveMovableSubsystem(
                                                        robotContainer.transporterSS,
                                                        () -> TransporterConstants.POWER))))));
    }

    @Override
    public boolean isFinished() {
        return shooter.isFinished();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("ttt", ttt::atSetpoint, null);
        builder.addBooleanProperty("shooter", shooter::atSetpoint, null);
        builder.addDoubleProperty("balls", shooter::getBallsShot, null);
        builder.addBooleanProperty("pitcher", pitcher::atSetpoint, null);
    }
}
