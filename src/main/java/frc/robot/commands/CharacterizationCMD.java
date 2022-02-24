package frc.robot.commands;

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
     * Previous executions velocity for every component.
     */
    private double[] lastVelocities;
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
    }

    @Override
    public void execute() {
        if(state == CharacterizationState.RunningCycle) {
            characterizableSS.move(powers[cycle]);
            for(int i = 0; i < componentCount; i++) {
                double velocity = Math.abs(characterizableSS.getValues()[i]);
                //Check if the velocity is acceleration is within the acceleration tolerance.
                if(Math.abs(velocity - Math.abs(
                        lastVelocities[i])) < velocity * characterizationConstants.tolerancePercentage / 100) {
                    averageVelocities[i][cycle] += velocity;
                    sampleCount[i]++;
                } else {
                    averageVelocities[i][cycle] = 0;
                    sampleCount[i] = 0;
                }
            }
            //Checks if the cycle has finished.
            if(Math.abs(initialCharacterizationCyclePosition - characterizableSS.getCharacterizationCyclePosition())
                    >= characterizationConstants.cycleLength) {
                characterizableSS.stopMoving();
                state = CharacterizationState.FinishedCycle;
            }
        } else if(state == CharacterizationState.FinishedCycle) {
            //Calculates the average velocities.
            for(int i = 0; i < componentCount; i++) {
                if(sampleCount[i] > 0)
                    averageVelocities[i][cycle] /= sampleCount[i];
            }
            for(int i = 0; i < componentCount; i++)
                sampleCount[i] = 0;
            cycle++;
            state = CharacterizationState.Resetting;
        } else {
            //Checks if all the components have stopped moving.
            boolean hasStopped = true;
            for(double velocity : characterizableSS.getValues()) {
                if(velocity != 0) {
                    hasStopped = false;
                    break;
                }
            }
            if(hasStopped) {
                //Prepares the values for the next cycle.
                calculatePower();
                initialCharacterizationCyclePosition = characterizableSS.getCharacterizationCyclePosition();
                for(int i = 0; i < componentCount; i++) {
                    sampleCount[i] = 0;
                }
                state = CharacterizationState.RunningCycle;
            }
        }
        lastVelocities = characterizableSS.getValues();
    }

    private void calculatePower() {
        powers[cycle] =
                characterizationConstants.initialPower + characterizationConstants.powerIncrement * cycle;
        if(invertDirectionEveryCycle && cycle % 2 == 1)
            powers[cycle] = -powers[cycle];
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

    private enum CharacterizationState {
        /**
         * In this state the command moves the motor and sums up the velocity
         */
        RunningCycle,
        /**
         * In this state the command calculates the average velocities and calculates the new power
         */
        FinishedCycle,
        /**
         * In this state the command waits for the subsystem to stop moving, once it does, it calculates the new power.
         */
        Resetting
    }
}
