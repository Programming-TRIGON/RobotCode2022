package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDSS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class GenericPIDCMD extends CommandBase {
    private final PIDSS subsystem;
    private final BooleanSupplier isFinished;
    private final DoubleSupplier setpoint;

    public GenericPIDCMD(PIDSS subsystem, DoubleSupplier setpoint) {
        this(subsystem, setpoint, () -> false);
    }

    public GenericPIDCMD(PIDSS subsystem, DoubleSupplier setpoint, BooleanSupplier isFinished) {
        this.subsystem = subsystem;
        this.setpoint = setpoint;
        this.isFinished = isFinished;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setSetpoint(setpoint.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean();
    }
}
