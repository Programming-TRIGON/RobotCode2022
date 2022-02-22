package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
import frc.robot.utilities.LinearRegression;
;

public class CharacterizationCMD extends CommandBase {
    private final CharacterizableSubsystem characterizableSS;
    private final CharacterizationConstants characterizationConstants;
    private double[][] averageVelocities;
    private double[] lastVelocity;
    private double[] sampleCount;
    private double[] powers;
    private int cycle;
    private double initialCharacterizationCyclePosition;
    private CharacterizationState state;
    private boolean invertDirectionEveryCycle;
    private final int valueCount;

    /**
     * This command moves the swerve forward in backwards in order to calculate the
     * kA and kS values of every module and edits the Json file to match it.
     * In order to finalize this you must
     *
     * @param characterizableSS
     * @param invertDirectionEveryCycle If set to true the direction will flip after every cycle, this is useful is
     *                                  subsystems that have a limited amount of motion.
     */
    public CharacterizationCMD(
            CharacterizableSubsystem characterizableSS, boolean invertDirectionEveryCycle) {
        addRequirements(characterizableSS);
        this.characterizableSS = characterizableSS;
        this.characterizationConstants = characterizableSS.getCharacterizationConstants();
        this.invertDirectionEveryCycle = invertDirectionEveryCycle;

        valueCount = characterizableSS.getValues().length;
        averageVelocities = new double[valueCount][];
        lastVelocity = new double[valueCount];
        sampleCount = new double[valueCount];
        powers = new double[characterizationConstants.cycleCount];

        for(int i = 0; i < characterizationConstants.cycleCount; i++)
            averageVelocities[i] = new double[characterizationConstants.cycleCount];
    }

    public CharacterizationCMD(CharacterizableSubsystem characterizableSS) {
        this(characterizableSS, false);
    }

    @Override
    public void initialize() {
        cycle = 0;
        state = CharacterizationState.Resetting;
        for(int i = 0; i < valueCount; i++) {
            lastVelocity[i] = 0;
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
            for(int i = 0; i < valueCount; i++) {
                double velocity = Math.abs(characterizableSS.getValues()[i]);
                //Check if the velocity is acceleration is within the tolerance.
                if(Math.abs(velocity - Math.abs(
                        lastVelocity[i])) < velocity * characterizationConstants.tolerancePercentage / 100) {
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
            for(int i = 0; i < valueCount; i++) {
                if(sampleCount[i] > 0)
                    averageVelocities[i][cycle] /= sampleCount[i];
            }
            for(int i = 0; i < valueCount; i++)
                sampleCount[i] = 0;
            cycle++;
            state = CharacterizationState.Resetting;
        } else {
            //Checks if all the components have stopped moving.
            boolean hasStopped = true;
            for(double velocity : characterizableSS.getValues()) {
                if(velocity != 0)
                    hasStopped = false;
            }
            if(hasStopped) {
                //Prepares the values for the next cycle.
                calculatePower();
                initialCharacterizationCyclePosition = characterizableSS.getCharacterizationCyclePosition();
                for(int i = 0; i < valueCount; i++) {
                    sampleCount[i] = 0;
                }
                state = CharacterizationState.RunningCycle;
            }
        }
        lastVelocity = characterizableSS.getValues();
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
        double[] kV = new double[valueCount];
        double[] kS = new double[valueCount];
        for(int i = 0; i < valueCount; i++){
            LinearRegression linearRegression = new LinearRegression(averageVelocities[i],powers);
            kV[i] = linearRegression.slope();
            kS[i] = linearRegression.intercept();
        }
        characterizableSS.updateFeedforward(kV,kS);
        //TODO: decide whether it will automatically write to the Json file
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
         * In this state the command waits for the subsystem to stop moving and once it does calculates the new power
         */
        Resetting;
    }
}
