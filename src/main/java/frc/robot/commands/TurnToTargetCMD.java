package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.utilities.pid.PIDCoefs;
import frc.robot.utilities.pid.TrigonPIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TurnToTargetCMD extends CommandBase {
    private final SwerveSS swerveSS;
    private final DoubleSupplier measurement;
    private final BooleanSupplier inSight;
    private final double timeout;
    private final PIDController rotationPIDController;
    private double lastTimeSeenTarget;

    public TurnToTargetCMD(
            SwerveSS swerveSS, DoubleSupplier measurement, BooleanSupplier inSight, double setpoint, PIDCoefs coefs,
            double timeout) {
        this.swerveSS = swerveSS;
        this.measurement = measurement;
        this.inSight = inSight;
        this.timeout = timeout;

        rotationPIDController = new TrigonPIDController(coefs);
        rotationPIDController.setSetpoint(setpoint);

        addRequirements(swerveSS);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        lastTimeSeenTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(inSight.getAsBoolean()) {
            swerveSS.move(rotationPIDController.calculate(measurement.getAsDouble()));
            lastTimeSeenTarget = Timer.getFPGATimestamp();
        } else
            // The target wasn't found
            swerveSS.stopMoving();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSS.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > timeout);
    }

    public boolean atSetpoint() {
        return rotationPIDController.atSetpoint();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("List");

        builder.addBooleanProperty("Running", this::isScheduled, (x) -> {
            if(x)
                schedule();
        });
        builder.addDoubleProperty("error", rotationPIDController::getPositionError, null);
    }

    public void putOnDashboard(String entry) {
        SmartDashboard.putData(entry, this);
        SmartDashboard.putData(entry + "/PID", rotationPIDController);
    }
}
