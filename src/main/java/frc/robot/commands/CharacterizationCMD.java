package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
import frc.robot.utilities.LinearRegression;

public class CharacterizationCMD extends CommandBase {
    private final CharacterizableSubsystem characterizableSS;
    private final CharacterizationConstants characterizationConstants;
    /**
     * If set to true the direction will flip after every cycle, this is
     * useful issubsystems that have a limited amount of motion.
     */
    private final boolean invertDirectionEveryCycle;
    /**
     * Amount of components the code needs to generate kV and kS values for. This is based on the amount of values
     * the subsystems returns.
     */
    private final int componentCount;
    /**
     * A jagged array including the average velocity of evey cycle for every component
     * Use: averageVelocities[componentNumber][cycleNumber]
     * During cycles this is used temporarily as the sum of all previous velocities that were in the acceleration
     * tolerance in succession.
     */
    private final double[][] averageVelocities;
    /**
     * Amount of velocities summed in averageVelocities for every component.
     */
    private final double[] sampleCount;
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
    private double LastVelocitiesMeasurementTime;
    /**
     * Amount of cycles that have past
     */
    private int cycle;
    /**
     * The position of the subsystem at the beginning of the cycle. Used to tell how much the position has change to
     * know when to end the current cycle.
     */
    private double initialCharacterizationCyclePosition;

    /**
     * Current state of the command.
     */
    private CharacterizationState state;

    /**
     * This command moves a characterizable subsystem at different velocities in order to calculate the
     * kA and kS values of every module and edits the feedforward constants of the subsystem to match it.
     * In order to finalize this you must save and write to the Json file.
     *
     * @param characterizableSS         a characterizable subsystem.
     * @param invertDirectionEveryCycle If set to true the direction will flip after every cycle, this is useful is
     *                                  subsystems that have a limited amount of motion.
     */
    public CharacterizationCMD(
            CharacterizableSubsystem characterizableSS, boolean invertDirectionEveryCycle) {
        addRequirements(characterizableSS);
        this.characterizableSS = characterizableSS;
        this.characterizationConstants = characterizableSS.getCharacterizationConstants();
        this.invertDirectionEveryCycle = invertDirectionEveryCycle;

        componentCount = characterizableSS.getValues().length;
        averageVelocities = new double[componentCount][];
        lastVelocities = new double[componentCount];
        sampleCount = new double[componentCount];
        powers = new double[characterizationConstants.cycleCount];

        for(int i = 0; i < componentCount; i++)
            averageVelocities[i] = new double[characterizationConstants.cycleCount];
    }

    private enum CharacterizationState {
        /**
         * In this state the command moves the motor and sums up the velocity
         */
        Running,
        /**
         * In this state the command calculates the average velocities and calculates the new power
         */
        Finished,
        /**
         * In this state the command waits for the subsystem to stop moving, once it does, it calculates the new power.
         */
        Resetting
    }

    /**
     * This command moves a characterizable subsystem at different velocities in order to calculate the
     * kA and kS values of every module and edits the feedforward constants of the subsystem to match it.
     * In order to finalize this you must save and write to the Json file.
     *
     * @param characterizableSS a characterizable subsystem.
     */
    public CharacterizationCMD(CharacterizableSubsystem characterizableSS) {
        this(characterizableSS, false);
    }

    @Override
    public void initialize() {
        cycle = 0;
        state = CharacterizationState.Resetting;
        for(int i = 0; i < componentCount; i++) {
            lastVelocities[i] = 0;
            sampleCount[i] = 0;
            for(int j = 0; j < characterizationConstants.cycleCount; j++) {
                averageVelocities[i][j] = 0;
            }
        }
        LastVelocitiesMeasurementTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(state == CharacterizationState.Running)
            runningCycle();
        else if(state == CharacterizationState.Finished)
            finishingCycle();
        else
            resettingCycle();

        lastVelocities = characterizableSS.getValues();
        LastVelocitiesMeasurementTime = Timer.getFPGATimestamp();
    }

    private void runningCycle() {
        sumVelocities();
        if(hasFinishedCycle()) {
            characterizableSS.stopMoving();
            state = CharacterizationState.Finished;
        }
    }

    /**
     * Sums the velocities if the acceleration is within the acceleration tolerance
     */
    private void sumVelocities() {
        for(int i = 0; i < componentCount; i++) {
            double velocity = Math.abs(characterizableSS.getValues()[i]);
            if(isAcceptableAcceleration(i)) {
                averageVelocities[i][cycle] += velocity;
                sampleCount[i]++;
            } else {
                averageVelocities[i][cycle] = 0;
                sampleCount[i] = 0;
            }
        }
    }

    /**
     * @return If the velocity is acceleration is within the acceleration tolerance.
     */
    private boolean isAcceptableAcceleration(int componentIndex) {
        double velocity = Math.abs(characterizableSS.getValues()[componentIndex]);
        double acceleration = Math.abs(
                lastVelocities[componentIndex] - velocity) / (Timer.getFPGATimestamp() - LastVelocitiesMeasurementTime);
        return acceleration < velocity * characterizationConstants.tolerancePercentage / 100;
    }

    /**
     * @return If the cycle has finished.
     */
    private boolean hasFinishedCycle() {
        return Math.abs(initialCharacterizationCyclePosition - characterizableSS.getCharacterizationCyclePosition())
                >= characterizationConstants.cycleLength;
    }

    private void finishingCycle() {
        calculateAverageVelocities();
        cycle++;
        state = CharacterizationState.Resetting;
    }

    /**
     * Calculates the average velocities based on their summed value and sample count.
     */
    private void calculateAverageVelocities() {
        for(int i = 0; i < componentCount; i++) {
            if(sampleCount[i] > 0)
                averageVelocities[i][cycle] /= sampleCount[i];
        }
    }

    private void resettingCycle() {
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
        powers[cycle] =
                characterizationConstants.initialPower + characterizationConstants.powerIncrement * cycle;
        if(invertDirectionEveryCycle && cycle % 2 == 1)
            powers[cycle] = -powers[cycle];
        characterizableSS.move(powers[cycle]);
        initialCharacterizationCyclePosition = characterizableSS.getCharacterizationCyclePosition();
        for(int i = 0; i < componentCount; i++) {
            sampleCount[i] = 0;
        }
        state = CharacterizationState.Running;
    }

    @Override
    public void end(boolean interrupted) {
        characterizableSS.stopMoving();
        for(int i = 0; i < characterizationConstants.cycleCount; i++) {
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

    @Override
    public boolean isFinished() {
        return cycle >= characterizationConstants.cycleCount;
    }
}
