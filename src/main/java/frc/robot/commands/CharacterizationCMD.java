package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.CharacterizationConstants;
import frc.robot.subsystems.CharacterizableSubsystem;
;

public class CharacterizationCMD extends CommandBase {
    private final CharacterizableSubsystem characterizableSS;
    private final CharacterizationConstants characterizationConstants;
    private double[][] averageVelocities;
    private double[] lastVelocity;
    private double sampleCount;
    private double[] powers;
    private int cycle;
    private CharacterizationState state;
    private boolean invertDirectionEveryCycle;

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

        averageVelocities = new double[characterizableSS.getValues().length][];
        lastVelocity = new double[characterizableSS.getValues().length];
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
        sampleCount = 0;
        powers[cycle] = characterizationConstants.initialPower + characterizationConstants.powerIncrement * cycle;
        state = CharacterizationState.RunningCycle;
        for(int i = 0; i < characterizationConstants.cycleCount; i++) {
            lastVelocity[i] = 0;
            for(int j = 0; j < characterizationConstants.cycleCount; i++) {
                averageVelocities[i][j] = 0;
            }
        }
        calculatePower();
    }

    @Override
    public void execute() {
        if(state == CharacterizationState.FinishedCycle) {
            if(sampleCount > 0) {
                for(double[] velocities : averageVelocities) {
                    for(int i = 0; i < characterizationConstants.cycleCount; i++) {
                        // We save the sum of the velocities and then divide it by the sample count to
                        // get the average velocity
                        velocities[i] /= sampleCount;
                    }
                }
            }
            sampleCount = 0;
            cycle++;
            state = CharacterizationState.Resetting;
        } else {
        }
        characterizableSS.move(powers[cycle]);
    }

    private void calculatePower() {
        powers[cycle] =
                characterizationConstants.initialPower + characterizationConstants.powerIncrement * cycle;
        if(invertDirectionEveryCycle && cycle % 2 == 0)
            powers[cycle] = -powers[cycle];
    }

    @Override
    public void end(boolean interrupted) {
        characterizableSS.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return sampleCount <= 0 && cycle > characterizationConstants.cycleCount;
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
         * In this state the command waits for the subsystem to stop moving
         */
        Resetting;
    }
}
