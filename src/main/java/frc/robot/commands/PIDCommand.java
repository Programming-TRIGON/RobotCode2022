package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDFSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PIDCommand extends CommandBase {
    private final PIDFSubsystem subsystem;
    private final BooleanSupplier isFinished;
    private final DoubleSupplier setpoint;

    public PIDCommand(PIDFSubsystem subsystem, DoubleSupplier setpoint, BooleanSupplier isFinished) {
        this.subsystem = subsystem;
        this.setpoint = setpoint;
        this.isFinished = isFinished;

        addRequirements(subsystem);
    }

    public PIDCommand(PIDFSubsystem subsystem, DoubleSupplier setpoint) {
        this(subsystem, setpoint, () -> false);
    }

    @Override
    public void initialize() {
        subsystem.setSetpoint(setpoint.getAsDouble());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }

    public boolean atSetpoint() {
        return subsystem.atSetpoint();
    }
}
