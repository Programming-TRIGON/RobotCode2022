package frc.robot.commands.commandgroups;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveMovableSubsystem;
import frc.robot.commands.PIDCommand;
import frc.robot.commands.TurnToTargetCMD;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.LoaderConstants;
import frc.robot.constants.RobotConstants.TransporterConstants;
import frc.robot.subsystems.shooter.ShootingCalculations;

public class ShootCG extends ParallelCommandGroup {
    private final TurnToTargetCMD turnToTargetCMD;
    private final PIDCommand shooterCMD;
    private final PIDCommand pitcherCMD;

    public ShootCG(RobotContainer robotContainer) {
        turnToTargetCMD = new TurnToTargetCMD(
                robotContainer.swerveSS,
                () -> -robotContainer.hubLimelight.getTx(),
                robotContainer.hubLimelight::getTv,
                () -> -1.5 / ShootingCalculations.calculateDistance(robotContainer.hubLimelight.getTy()),
                RobotConstants.VisionConstants.HUB_TURN_TO_TARGET_COEFS, 1);
        shooterCMD = new PIDCommand(
                robotContainer.shooterSS,
                () -> ShootingCalculations.calculateVelocity(robotContainer.hubLimelight.getTy()));
        pitcherCMD = new PIDCommand(
                robotContainer.pitcherSS,
                () -> ShootingCalculations.calculateAngle(robotContainer.hubLimelight.getTy()));

        turnToTargetCMD.putOnDashboard("ShootCG/TurnToTargetCMD");

        addCommands(
                shooterCMD,
                pitcherCMD,
                turnToTargetCMD,
                new WaitCommand(0.3).andThen(
                        new ParallelCommandGroup(
                                new WaitUntilCommand(
                                        () -> shooterCMD.atSetpoint() && robotContainer.shooterSS.getSetpoint() > 0),
                                new WaitUntilCommand(turnToTargetCMD::atSetpoint)
                        ).andThen(
                                new ParallelCommandGroup(
                                        new MoveMovableSubsystem(robotContainer.loaderSS, () -> LoaderConstants.POWER),
                                        new WaitCommand(RobotConstants.ShooterConstants.TRANSPORTER_WAIT_TIME).andThen(
                                                new SequentialCommandGroup(
                                                        new ParallelCommandGroup(
                                                                new WaitUntilCommand(
                                                                        () -> shooterCMD.atSetpoint() && robotContainer.shooterSS.getSetpoint() > 0),
                                                                new WaitUntilCommand(turnToTargetCMD::atSetpoint)),
                                                        new MoveMovableSubsystem(
                                                                robotContainer.transporterSS,
                                                                () -> TransporterConstants.POWER)
                                                ))))

                ));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("turnToTarget", turnToTargetCMD::atSetpoint, null);
        builder.addBooleanProperty("shooter", shooterCMD::atSetpoint, null);
        builder.addBooleanProperty("pitcher", pitcherCMD::atSetpoint, null);
    }
}
