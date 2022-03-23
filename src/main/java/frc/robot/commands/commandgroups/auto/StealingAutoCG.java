package frc.robot.commands.commandgroups.auto;

import frc.robot.RobotContainer;
import frc.robot.commands.commandgroups.IntakeCG;
import frc.robot.subsystems.swerve.SupplierDriveCMD;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class StealingAutoCG extends AutoCG {
    public StealingAutoCG(RobotContainer robotContainer, Supplier<Boolean> isStartingLeft) {
        super(robotContainer);
        this.robotContainer = robotContainer;
        startingAngle = 0;
        DoubleSupplier speedSignum = () -> isStartingLeft.get() ? 1 : -1;
        addCommands(
                getStartingCommands(),
                new IntakeCG(robotContainer).withTimeout(1.5),
                new SupplierDriveCMD(robotContainer.swerveSS, () -> 0.0, () -> -0.35, () -> 0.0, false).withTimeout(
                        1.5),
                new AutoShootCG(robotContainer, () -> speedSignum.getAsDouble() * 0.3).withTimeout(4.5),
                new AutoCollectCG(robotContainer, () -> speedSignum.getAsDouble() * -0.35).withTimeout(3),
                new AutoShootCG(robotContainer, () -> speedSignum.getAsDouble() * 0.3));
        //TODO: finish
    }
}
