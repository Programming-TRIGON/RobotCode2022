package frc.robot.utilities;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.constants.RobotConstants;

/**
 * A class that has values that can be saved and loaded from a file.
 */
public interface Saveable extends Sendable {
    void save();

    void load();

    default void write() {
        save();
        RobotConstants.write();
    }

    @Override
    default void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("load", () -> false, (load) -> load());
        builder.addBooleanProperty("write", () -> false, (write) -> write());
    }
}
