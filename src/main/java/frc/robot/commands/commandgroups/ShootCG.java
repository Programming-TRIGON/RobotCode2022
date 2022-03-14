package frc.robot.commands.commandgroups;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    GenericTurnToTargetCMD ttt;
    PIDCommand shooter, pitcher;

    public ShootCG(RobotContainer robotContainer) {
        GenericTurnToTargetCMD turnToTargetCMD = new GenericTurnToTargetCMD(
                robotContainer.swerveSS,
                () -> -robotContainer.hubLimelight.getTx(),
                robotContainer.hubLimelight::getTv,
                0,
                RobotConstants.VisionConstants.HUB_TTT_COEFS,
                1);
        PIDCommand shooterCMD = new PIDCommand(
                robotContainer.shooterSS,
                () -> ShootingCalculations.calculateVelocity(robotContainer.hubLimelight.getTy()));
        PIDCommand pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS,
                () -> ShootingCalculations.calculateAngle(robotContainer.hubLimelight.getTy()));

        ttt = turnToTargetCMD;
        shooter = shooterCMD;
        pitcher = pitcherCMD;

        turnToTargetCMD.putOnDashboard("ShootCG/TTTCMD");

        addCommands(
                shooterCMD,
                pitcherCMD,
                turnToTargetCMD,
                new WaitUntilCommand(
                        () -> shooterCMD.atSetpoint() && pitcherCMD.atSetpoint() && turnToTargetCMD.atSetpoint()
                ).andThen(
                        new ParallelCommandGroup(
                                new MoveMovableSubsystem(robotContainer.loaderSS, () -> LoaderConstants.POWER),
                                new WaitCommand(RobotConstants.ShooterConstants.TRANSPORTER_WAIT_TIME).andThen(
                                        new MoveMovableSubsystem(
                                                robotContainer.transporterSS, () -> TransporterConstants.POWER)))));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("ttt", ttt::atSetpoint, null);
        builder.addBooleanProperty("shooter", shooter::atSetpoint, null);
        builder.addBooleanProperty("pitcher", pitcher::atSetpoint, null);
    }
}
