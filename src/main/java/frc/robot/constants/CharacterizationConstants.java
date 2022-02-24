package frc.robot.constants;

public class CharacterizationConstants {
    public final double initialPower;
    public final double powerIncrement;
    public final int cycleCount;
    public final double cycleLength;
    public final double tolerancePercentage;

    /**
     * @param initialPower        Starting power between -1 and 1 used in the initial state.
     * @param powerIncrement      Amount of power to change in every cycle
     * @param cycleCount          Amount of times to increment the power and repeat the test. The higher the value the
     *                            more accurate the result.
     * @param cycleLength         Length of every cycle. Units should match the units used in the
     * @param tolerancePercentage Allowable percentage of the velocity that can accelerate. This is used in order to
     *                            ignore the beginning acceleration when calculating the average velocity.
     */
    public CharacterizationConstants(
            double initialPower, double powerIncrement, int cycleCount, double cycleLength,
            double tolerancePercentage) {
        this.initialPower = initialPower;
        this.powerIncrement = powerIncrement;
        this.cycleCount = cycleCount;
        this.cycleLength = cycleLength;
        this.tolerancePercentage = tolerancePercentage;
    }
}