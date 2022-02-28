package frc.robot.motion_profiling;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.constants.RobotConstants.MotionProfilingConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSS;
import frc.robot.utilities.DriverStationLogger;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

/**
 * Class for creating a new path for motion profiling path following.
 */
public class Path {
    private static final double kDefaultStartPathVelocity = 0.0;
    private static final double kDefaultEndPathVelocity = 0.0;
    private Trajectory trajectory;
    private boolean reversed;

    /**
     * Creates new path for motion profiling with default configuration.
     *
     * @param waypoints the path waypoints
     */
    public Path(DrivetrainSS drivetrainSS, MotionProfilingConstants motionProfilingConstants, Waypoint... waypoints) {
        this(drivetrainSS, motionProfilingConstants, false, kDefaultStartPathVelocity, kDefaultEndPathVelocity,
                waypoints);
    }

    /**
     * Creates new path for motion profiling with velocity default configuration.
     *
     * @param reversed  whether the path is reversed
     * @param waypoints the path waypoints
     */
    public Path(DrivetrainSS drivetrainSS, MotionProfilingConstants motionProfilingConstants, boolean reversed,
                Waypoint... waypoints) {
        this(drivetrainSS, motionProfilingConstants, reversed, kDefaultStartPathVelocity, kDefaultEndPathVelocity,
                waypoints);
    }

    /**
     * Creates new path for motion profiling with start velocity default value of
     * zero.
     *
     * @param reversed    whether the path is reversed
     * @param endVelocity the velocity at the end of the path
     * @param waypoints   the path waypoints
     */
    public Path(DrivetrainSS drivetrainSS, MotionProfilingConstants motionProfilingConstants, boolean reversed,
                double endVelocity, Waypoint... waypoints) {
        this(drivetrainSS, motionProfilingConstants, reversed, kDefaultStartPathVelocity, endVelocity, waypoints);
    }

    /**
     * Creates new path with reversed, start velocity & end velocity params.
     *
     * @param reversed      whether the path is reversed
     * @param startVelocity the velocity at the start of the path
     * @param endVelocity   the velocity at the end of the path
     * @param waypoints     the path waypoints
     */
    public Path(DrivetrainSS drivetrainSS, MotionProfilingConstants motionProfilingConstants, boolean reversed,
                double startVelocity, double endVelocity, Waypoint... waypoints) {
        this.reversed = reversed;
        TrajectoryConfig config = new TrajectoryConfig(motionProfilingConstants.MAX_VELOCITY,
                motionProfilingConstants.MAX_ACCELERATION)
                .addConstraint(new CentripetalAccelerationConstraint(motionProfilingConstants.MAX_CENTRIPETAL_ACCELERATION))
                .setKinematics(drivetrainSS.getKinematics()).setReversed(reversed).setStartVelocity(startVelocity)
                .setEndVelocity(endVelocity);

        trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints), config);
    }

    /**
     * @param pathName the name of the path to load from the filesystem.
     */
    public Path(String pathName) {
        var path = Paths.get(Filesystem.getDeployDirectory() + "/paths/output/" + pathName);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            DriverStationLogger.logErrorToDS("Could not load " + pathName + " path from: " + path.toString()
                    + "\nInitializing with an empty path");
            trajectory = new Trajectory(List.of(new Trajectory.State()));
        }
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }

    public boolean isReversed() {
        return reversed;
    }

    /**
     * Return the time of the robot to drive the path.
     *
     * @return path time in seconds
     */
    public double getPathTime() {
        return trajectory.getTotalTimeSeconds();
    }
}
