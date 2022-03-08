package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
import frc.robot.utilities.LinearRegression;

public class CharacterizationCMD extends CommandBase {
    private final CharacterizableSubsystem characterizableSS;
    private final CharacterizationConstants constants;
    /**
     * The Amount of components that the code needs to generate kV and kS values for. This is set to the size of the
     * values array that the subsystem returns.
     */
    private final int componentCount;
    /**
     * A jagged array including the average velocity of evey cycle for every component.
     * Use: averageVelocities[componentIndex][cycleIndex].
     */
    private final double[][] averageVelocities;
    /**
     * This is used as the last sum of all previous velocities that were in the acceleration
     * tolerance in succession.
     */
    private final double[] velocitySums;
    /**
     * Amount of velocities summed in averageVelocities for every component.
     */
    private final double[] sampleCounts;
    /**
     * Power used in every cycle.
     */
    private final double[] powers;
    /**
     * Previous execution's velocity for every component.
     */
    private double[] lastVelocities;
    /**
     * Time when the lastVelocities were measured. Used in order to calculate the acceleration.
     */
    private double lastVelocitiesMeasurementTime;
    /**
     * Amount of cycles that have been completed.
     */
    private int completedCycles;
    /**
     * The position of the subsystem in space or time at the beginning of the cycle. Used to tell how much the
     * position has changed to know when to end the current cycle.
     */
    private double initialCyclePosition;
    /**
     * Current state of the command.
     */
    private CharacterizationState state;

    /**
     * This command moves a characterizable subsystem at different velocities in order to calculate the
     * kV and kS values of every module and edits the feedforward constants of the subsystem to match it.
     * In order to finalize this you must save and write to the Json file.
     *
     * @param characterizableSS A characterizable subsystem.
     */
    public CharacterizationCMD(
            CharacterizableSubsystem characterizableSS) {
        this.characterizableSS = characterizableSS;
        this.constants = characterizableSS.getCharacterizationConstants();

        componentCount = characterizableSS.getValues().length;
        averageVelocities = new double[componentCount][];
        velocitySums = new double[componentCount];
        lastVelocities = new double[componentCount];
        sampleCounts = new double[componentCount];
        powers = new double[constants.cycleCount];

        for(int i = 0; i < componentCount; i++)
            averageVelocities[i] = new double[constants.cycleCount];

        addRequirements(characterizableSS);
    }

    @Override
    public void initialize() {
        completedCycles = 0;
        state = CharacterizationState.Resetting;
        for(int i = 0; i < componentCount; i++) {
            lastVelocities[i] = 0;
            sampleCounts[i] = 0;
            for(int j = 0; j < constants.cycleCount; j++) {
                averageVelocities[i][j] = 0;
            }
        }
        lastVelocitiesMeasurementTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        switch(state) {
            case Running:
                runCycle();
                break;
            case Finished:
                finishCycle();
                break;
            case Resetting:
                resetCycle();
                break;
        }

        lastVelocities = characterizableSS.getValues();
        lastVelocitiesMeasurementTime = Timer.getFPGATimestamp();
    }

    /**
     * Sums up the velocities for every component.
     */
    private void runCycle() {
        sumVelocities();
        if(isCycleFinished()) {
            characterizableSS.stopMoving();
            state = CharacterizationState.Finished;
        }
    }

    /**
     * Adds the current velocity to the sum if the acceleration is within the acceleration tolerance, else it resets
     * the sum.
     */
    private void sumVelocities() {
        for(int i = 0; i < componentCount; i++) {
            double velocity = Math.abs(characterizableSS.getValues()[i]);
            if(isWithinAccelerationTolerance(i)) {
                velocitySums[i] += velocity;
                sampleCounts[i]++;
            } else {
                velocitySums[i] = 0;
                sampleCounts[i] = 0;
            }
        }
    }

    /**
     * @param componentIndex Index of the component to check the acceleration of.
     * @return Whether the acceleration is within the acceleration tolerance.
     */
    private boolean isWithinAccelerationTolerance(int componentIndex) {
        double velocity = Math.abs(characterizableSS.getValues()[componentIndex]);
        double acceleration =
                Math.abs(lastVelocities[componentIndex] - velocity)
                        / (Timer.getFPGATimestamp() - lastVelocitiesMeasurementTime);
        return acceleration < velocity * constants.tolerancePercentage / 100;
    }

    /**
     * @return Whether the cycle has finished.
     */
    private boolean isCycleFinished() {
        return Math.abs(initialCyclePosition - characterizableSS.getCharacterizationCyclePosition())
                >= constants.cycleLength;
    }

    /**
     * Calculates the average velocities of every component.
     */
    private void finishCycle() {
        calculateAverageVelocities();
        completedCycles++;
        state = CharacterizationState.Resetting;
    }

    /**
     * Calculates the average velocities based on their summed value and sample count.
     */
    private void calculateAverageVelocities() {
        for(int i = 0; i < componentCount; i++) {
            if(sampleCounts[i] > 0)
                averageVelocities[i][completedCycles] = velocitySums[i] / sampleCounts[i];
        }
    }

    /**
     * Waits for the subsystem to stop moving, once it does, it calculates the new
     * power, moves the motor at that power, and sets up the next cycle.
     */
    private void resetCycle() {
        if(hasSystemStopped())
            setupNextCycle();
    }

    /**
     * Checks if all the components have stopped moving.
     */
    private boolean hasSystemStopped() {
        for(double velocity : characterizableSS.getValues()) {
            if(velocity != 0)
                return false;
        }
        return true;
    }

    /**
     * Prepares the values for the next cycle.
     */
    private void setupNextCycle() {
        powers[completedCycles] =
                constants.initialPower + constants.powerIncrement * completedCycles;
        if(constants.invertDirectionEveryCycle && completedCycles % 2 == 1)
            powers[completedCycles] = -powers[completedCycles];
        characterizableSS.move(powers[completedCycles]);
        initialCyclePosition = characterizableSS.getCharacterizationCyclePosition();
        for(int i = 0; i < componentCount; i++) {
            velocitySums[i] = 0;
            sampleCounts[i] = 0;
        }
        state = CharacterizationState.Running;
    }

    @Override
    public void end(boolean interrupted) {
        characterizableSS.stopMoving();
        if(!interrupted) {
            for(int i = 0; i < constants.cycleCount; i++) {
                powers[i] = Math.abs(powers[i]);
            }
            double[] kV = new double[componentCount];
            double[] kS = new double[componentCount];
            for(int i = 0; i < componentCount; i++) {
                LinearRegression linearRegression = new LinearRegression(averageVelocities[i], powers);
                kV[i] = linearRegression.slope();
                kS[i] = linearRegression.intercept();
            }
            characterizableSS.updateFeedforward(kV, kS);
        }
    }

    @Override
    public boolean isFinished() {
        return completedCycles >= constants.cycleCount;
    }

    private enum CharacterizationState {
        /**
         * In this state the command moves the motor and sums up the velocities for every component.
         */
        Running,
        /**
         * In this state the command calculates the average velocities of every component.
         */
        Finished,
        /**
         * In this state the command waits for the subsystem to stop moving, once it does, it calculates the new
         * power, moves the motor at that power, and sets up the next cycle.
         */
        Resetting
    }
}
