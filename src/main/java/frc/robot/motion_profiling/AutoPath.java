package frc.robot.motion_profiling;

import frc.robot.constants.RobotConstants.MotionProfilingConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSS;

/**
 * This enum represent and hold the instances of auto paths
 */
public enum AutoPath {
    FacingPowerPortToTrenchStart, InLineWithTrenchToTrenchStart, InTrench, ReverseInTrench, RightOfPortToMiddleField,
    FacingPowerPortToMiddleField, InitLineToEnemyTrench, EnemyTrenchToPort, SimpleAutoToTrench, TurnFromTrenchToPort,
    Test(new Waypoint(0, 0, 0), new Waypoint(0, 1, 0)),
    AutoNavBarrel(
            new Waypoint(2.145145583, 1.230035605, 0),
            new Waypoint(1.759729355, 4.526358605, 0),
            new Waypoint(1.181605014, 3.157116743, 0),
            new Waypoint(2.013292663, 3.654100826, 0),
            new Waypoint(3.139113749, 7.041706616, 0),
            new Waypoint(3.382534524, 5.378331318, 0),
            new Waypoint(1.384455660, 6.483867340, 0),
            new Waypoint(0.806331318, 7.751683878, 0),
            new Waypoint(2.084290389, 8.390663413, 0),
            new Waypoint(2.743554989, 1.483598913, 0)),
    AutoNavBounce(
            new Waypoint(2.34799622871721, 1.2807482668737173, 0),
            new Waypoint(2.734422985247697, 2.004575919603169, 0),
            new Waypoint(3.773527282097533, 2.363314307801327, 0),
            new Waypoint(2.3942400309218583, 2.536498357276299, 0),
            new Waypoint(0.9531013335051219, 3.2972711460413575, 0),
            new Waypoint(0.8355835856471052, 4.274523996650132, 0),
            new Waypoint(3.773527282097533, 4.596151517103652, 0),
            new Waypoint(1.2314328415898994, 4.954889905301809, 0),
            new Waypoint(1.052063647490821, 6.525916639824776, 0),
            new Waypoint(3.736416414352896, 6.8970253172711455, 0),
            new Waypoint(2.5488686465245123, 7.193912259228242, 0),
            new Waypoint(2.206000776440575, 8.29938062226166, 0)),
    AutoNavSlalom(
            new Waypoint(2.34799622871721, 1.2807482668737173, 0),
            new Waypoint(2.734422985247697, 2.004575919603169, 0),
            new Waypoint(3.773527282097533, 2.363314307801327, 0),
            new Waypoint(2.3942400309218583, 2.536498357276299, 0),
            new Waypoint(0.9531013335051219, 3.2972711460413575, 0),
            new Waypoint(0.8355835856471052, 4.274523996650132, 0),
            new Waypoint(3.773527282097533, 4.596151517103652, 0),
            new Waypoint(1.2314328415898994, 4.954889905301809, 0),
            new Waypoint(1.052063647490821, 6.525916639824776, 0),
            new Waypoint(3.736416414352896, 6.8970253172711455, 0),
            new Waypoint(2.5488686465245123, 7.193912259228242, 0),
            new Waypoint(2.206000776440575, 8.29938062226166, 0)
    );

    private final Path path;
    private final Waypoint[] waypoints;

    AutoPath(Waypoint... waypoints) {
        this.path = null;
        this.waypoints = waypoints;
    }

    AutoPath() {
        path = new Path(name() + ".wpilib.json");
        this.waypoints = null;
    }

    AutoPath(String name) {
        path = new Path(name + ".wpilib.json");
        this.waypoints = null;
    }

    public Path getPath(DrivetrainSS drivetrainSS, MotionProfilingConstants constants) {
        if (path == null)
            return new Path(drivetrainSS, constants, waypoints);
        else
            return path;
    }
}
