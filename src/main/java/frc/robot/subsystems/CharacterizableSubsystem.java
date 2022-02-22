package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.CharacterizationConstants;

/**
 * Characterizable subsystem is a subsystem the includes a component that uses a feedforward value that needs to be
 * calibrated. The move function should directly set the power of all motors in the system. getValues function should
 * return the velocity of the motors. updateFeedforward should set the feedforward in the same order.
 */
public interface CharacterizableSubsystem extends TestableSubsystem {

    /**
     * sets the feedforward for all the different components of the subsystem
     *
     * @param kV velocity gains
     * @param kS static gains
     */
    void updateFeedforward(double[] kV, double[] kS);

    /**
     * @return characterization constants of the subsystem to be used in the CharacterizationCMD.
     */
    CharacterizationConstants getCharacterizationConstants();

    /**
     * This function returns a value that the CharacterizationCMD tracks in order to know when to finish each test
     * cycle.
     * By default, it is set to time. If the system has a limited amount of motion a length limit can be added by
     * returning the position of the system.
     *
     * @return the current value which the CharacterizationCMD tracks in order to know when to finish each test.
     */
    default double getCharacterizationCyclePosition() {
        return Timer.getFPGATimestamp();
    }
}
