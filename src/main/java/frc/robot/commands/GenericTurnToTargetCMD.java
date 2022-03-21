package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSS;
import frc.robot.utilities.pid.PIDCoefs;
import frc.robot.utilities.pid.TrigonPIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * This is just template for a subsystem turn to target command. It will
 * probably be changed according the game and the robot.
 */
public class GenericTurnToTargetCMD extends CommandBase {
    private final SwerveSS swerveSS;
    private final DoubleSupplier measurement;
    private final BooleanSupplier inSight;
    private final double timeout;
    private final TrigonPIDController rotationPIDController;
    private final DoubleSupplier setpoint;
    private double lastTimeSeenTarget;
    private boolean running;

    public GenericTurnToTargetCMD(
            SwerveSS swerveSS, DoubleSupplier measurement, BooleanSupplier inSight, DoubleSupplier setpoint,
            PIDCoefs coefs,
            double timeout) {
        this.swerveSS = swerveSS;
        this.measurement = measurement;
        this.inSight = inSight;
        this.timeout = timeout;
        this.setpoint = setpoint;

        rotationPIDController = new TrigonPIDController(coefs);

        addRequirements(swerveSS);
    }

    @Override
    public void initialize() {
        rotationPIDController.reset();
        lastTimeSeenTarget = Timer.getFPGATimestamp();
        running = true;
    }

    @Override
    public void execute() {
        rotationPIDController.setSetpoint(setpoint.getAsDouble());
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
        running = false;
    }

    @Override
    public boolean isFinished() {
        return ((Timer.getFPGATimestamp() - lastTimeSeenTarget) > timeout) || atSetpoint();
    }

    public boolean atSetpoint() {
        return rotationPIDController.atSetpoint();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("List");

        builder.addBooleanProperty("Running", () -> running, (x) -> {
            if(x)
                schedule();
        });
        builder.addDoubleProperty(
                "error", () -> Math.abs(rotationPIDController.getSetpoint() - measurement.getAsDouble()), null);
    }

    public void putOnDashboard(String entry) {
        SmartDashboard.putData(entry, this);
        SmartDashboard.putData(entry + "/PID", rotationPIDController);
    }
}
