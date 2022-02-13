package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.RobotConstants.SwerveConstants.CharacterizationConstants;;

public class SwerveCharacterizationCMD extends CommandBase {
    private final SwerveSS swerveSS;
    private double[][] averageVelocities;
    private double sampleCount;
    private double[] powers;
    private int test;
    private boolean hasCompletedSample;

    /**
     * This command moves the swerve forward in backwards in order to calculate the
     * kA and kS values of every module and edits the Json file to match it.
     * In order to finalize this you must 
     * 
     * @param swerveSS
     */
    public SwerveCharacterizationCMD(SwerveSS swerveSS) {
        addRequirements(swerveSS);
        this.swerveSS = swerveSS;
        averageVelocities = new double[4][];
        powers = new double[CharacterizationConstants.TEST_COUNT];
        for (double[] velocities : averageVelocities) {
            velocities = new double[CharacterizationConstants.TEST_COUNT];
            for (int i = 0; i < CharacterizationConstants.TEST_COUNT; i++) {
                velocities[i] = 0;
            }
        }
    }

    @Override
    public void initialize() {
        test = 0;
        sampleCount = 0;
        powers[test] = CharacterizationConstants.INITIAL_POWER + CharacterizationConstants.POWER_INCREMENT * test;
        hasCompletedSample = false;
    }

    @Override
    public void execute() {
        if (hasCompletedSample) {
            if (sampleCount > 0) {
                for (double[] velocities : averageVelocities) {
                    for (int i = 0; i < CharacterizationConstants.TEST_COUNT; i++) {
                        // We save the sum of the velocities and then divide it by the sample count to
                        // get the average velocity
                        velocities[i] /= sampleCount;
                    }
                }
                sampleCount = 0;
                test++;
            } else {
                powers[test] = CharacterizationConstants.INITIAL_POWER
                        + CharacterizationConstants.POWER_INCREMENT * test;
                hasCompletedSample = false;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        swerveSS.stopMoving();
    }

    @Override
    public boolean isFinished() {
        return sampleCount <= 0 && test > CharacterizationConstants.TEST_COUNT;
    }
}
