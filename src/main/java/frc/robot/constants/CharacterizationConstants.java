package frc.robot.constants;

public class CharacterizationConstants {
    public final double initialPower;
    public final double powerIncrement;
    public final int cycleCount;
    public final double cycleLength;
    public final double tolerancePercentage;
    public final boolean invertDirectionEveryCycle;

    /**
     * @param initialPower              Starting power between 0 and 1 used in the initial state.
     * @param powerIncrement            Amount of power to change in every cycle
     * @param cycleCount                Amount of times to increment the power and repeat the test. The higher the
     *                                  value the
     *                                  more accurate the result.
     * @param cycleLength               Length of every cycle. Units should match the units used in the
     * @param tolerancePercentage       Allowable percentage of the velocity that can accelerate. This is used in
     *                                  order to
     *                                  ignore the beginning acceleration when calculating the average velocity.
     * @param invertDirectionEveryCycle If set to true the direction will flip after every cycle, this is
     *                                  useful for subsystems that have a limited amount of motion.
     */

    public CharacterizationConstants(
            double initialPower, double powerIncrement, int cycleCount, double cycleLength,
            double tolerancePercentage, boolean invertDirectionEveryCycle) {
        this.initialPower = Math.abs(initialPower);
        this.powerIncrement = powerIncrement;
        this.cycleCount = cycleCount;
        this.cycleLength = cycleLength;
        this.tolerancePercentage = tolerancePercentage;
        this.invertDirectionEveryCycle = invertDirectionEveryCycle;
    }
}