package frc.robot.motion_profiling;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.RobotConstants.MotionProfilingConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSS;

public class TrigonSwerveControllerCMDGP extends SequentialCommandGroup {
    /**
     * A command that uses two PID controllers ({@link PIDController}) and a
     * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
     * {@link Trajectory} with a swerve drive.
     *
     * <p>
     * This command outputs the raw desired Swerve Module States
     * ({@link SwerveModuleState}) in an array. The desired wheel and module
     * rotation velocities should be taken from those and used in velocity PIDs.
     *
     * <p>
     * The robot angle controller does not follow the angle given by the trajectory
     * but rather goes to the angle given in the final state of the trajectory.
     */
    public TrigonSwerveControllerCMDGP(DrivetrainSS drivetrainSS, MotionProfilingConstants constants, AutoPath path) {
        constants.THETA_PROFILED_PID_CONTROLLER.enableContinuousInput(-180, 180);
        constants.THETA_PROFILED_PID_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        SmartDashboard.putData("TrigonSwerveControllerCMDGP/PID X", constants.X_PID_CONTROLLER);
        SmartDashboard.putData("TrigonSwerveControllerCMDGP/PID Y", constants.Y_PID_CONTROLLER);
        SmartDashboard.putData("TrigonSwerveControllerCMDGP/PID Rotation", constants.THETA_PROFILED_PID_CONTROLLER);
        addCommands(
                new InstantCommand(() ->
                {
                    drivetrainSS.stopMoving();
                    drivetrainSS.SetSpeedMotorRampRates(0);
                    drivetrainSS.resetOdometry(
                            path.getPath(drivetrainSS, constants).getTrajectory().getInitialPose()
                    );
                }, drivetrainSS),
                new SwerveControllerCommand(
                        path.getPath(drivetrainSS, constants).getTrajectory(),
                        drivetrainSS::getPose,
                        drivetrainSS.getKinematics(),
                        constants.X_PID_CONTROLLER,
                        constants.Y_PID_CONTROLLER,
                        constants.THETA_PROFILED_PID_CONTROLLER,
                        drivetrainSS::setDesiredStates,
                        drivetrainSS
                ),
                new InstantCommand(() -> {
                    drivetrainSS.SetSpeedMotorRampRates();
                    drivetrainSS.stopMoving();
                }, drivetrainSS)
        );
    }
}
