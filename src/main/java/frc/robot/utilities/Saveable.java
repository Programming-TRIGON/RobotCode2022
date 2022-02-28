package frc.robot.utilities;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.constants.RobotConstants;

/**
 * A class that has values that can be saved and loaded from a file.
 */
public interface Saveable extends Sendable {
    /**
     * A method that saves the class' values to the LOCAL_CONSTANTS object.
     */
    void save();

    /**
     * A method that reloads the class' values from the LOCAL_CONSTANTS object.
     */
    void load();

    /**
     * A function that writes LOCAL_CONSTANTS to the json file.
     */
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
