package frc.robot.subsystems.shooter;

import frc.robot.constants.RobotConstants.HubLimelightConstants;

import java.util.ArrayList;
import java.util.List;

public class ShootingCalculations {
    //TODO: Set zone limits and coefs
    private static final ArrayList<ShooterWaypoint> SHOOTER_WAYPOINTS = new ArrayList<ShooterWaypoint>(List.of(
            new ShooterWaypoint(0.744, 7, 2700),
            new ShooterWaypoint(1.11, 9, 3000),
            new ShooterWaypoint(1.98, 11, 3000),
            new ShooterWaypoint(2.75, 19, 3200),
            new ShooterWaypoint(3.1, 20, 3400),
            new ShooterWaypoint(4.1, 20, 3800),
            new ShooterWaypoint(4.4, 20, 4100)
    ));

    /**
     * Calculates the optimal angle of the pitcher
     * based on the zone the robot is in.
     *
     * @param y the value on the y-axis that the limelight sees the target
     * @return target angle of the pitcher
     */
    public static double calculateAngle(double y) {
        if(SHOOTER_WAYPOINTS.size() == 0)
            return 0;
        else if(SHOOTER_WAYPOINTS.size() == 1)
            return SHOOTER_WAYPOINTS.get(0).getAngle();

        double distance = calculateDistance(y);

        // If the distance is smaller than all the points,
        // make a slope using the first two points.
        ShooterWaypoint waypoint0 = SHOOTER_WAYPOINTS.get(0);
        ShooterWaypoint waypoint1 = SHOOTER_WAYPOINTS.get(1);

        // In ShooterZone we sort the shooter waypoints,
        // so we can use it as a sorted array from closest to farthest.
        for(int i = 1; i < SHOOTER_WAYPOINTS.size() && SHOOTER_WAYPOINTS.get(i).getDistance() < distance; i++) {
            // If the distance is bigger than only the max point,
            // make a slope using the last two points.
            if(i + 1 >= SHOOTER_WAYPOINTS.size()) {
                waypoint0 = SHOOTER_WAYPOINTS.get(SHOOTER_WAYPOINTS.size() - 2);
                waypoint1 = SHOOTER_WAYPOINTS.get(SHOOTER_WAYPOINTS.size() - 1);
            } else {
                waypoint0 = SHOOTER_WAYPOINTS.get(i);
                waypoint1 = SHOOTER_WAYPOINTS.get(i + 1);
            }
        }

        double deltaA = waypoint1.getAngle() - waypoint0.getAngle();
        double deltaD = waypoint1.getDistance() - waypoint0.getDistance();
        double slope = deltaA / deltaD;
        double intercept = waypoint0.getAngle() - slope * waypoint0.getDistance();

        return slope * distance + intercept;
    }

    /**
     * Calculates the optimal velocity of the shooter
     * based on the zone the robot is in using a slope between two points.
     *
     * @return target velocity of the shooter
     */
    public static double calculateVelocity(double y) {
        if(SHOOTER_WAYPOINTS.size() == 0)
            return 0;
        else if(SHOOTER_WAYPOINTS.size() == 1)
            return SHOOTER_WAYPOINTS.get(0).getVelocity();

        double distance = calculateDistance(y);

        // If the distance is smaller than all the points,
        // make a slope using the first two points.
        ShooterWaypoint waypoint0 = SHOOTER_WAYPOINTS.get(0);
        ShooterWaypoint waypoint1 = SHOOTER_WAYPOINTS.get(1);

        // In ShooterZone we sort the shooter waypoints,
        // so we can use it as a sorted array from closest to farthest.
        for(int i = 1; SHOOTER_WAYPOINTS.get(i).getDistance() < distance; i++) {
            // If the distance is bigger than only the max point,
            // make a slope using the last two points.
            if(i + 1 >= SHOOTER_WAYPOINTS.size()) {
                waypoint0 = SHOOTER_WAYPOINTS.get(SHOOTER_WAYPOINTS.size() - 2);
                waypoint1 = SHOOTER_WAYPOINTS.get(SHOOTER_WAYPOINTS.size() - 1);
            } else {
                waypoint0 = SHOOTER_WAYPOINTS.get(i);
                waypoint1 = SHOOTER_WAYPOINTS.get(i + 1);
            }
        }

        double deltaV = waypoint1.getVelocity() - waypoint0.getVelocity();
        double deltaD = waypoint1.getDistance() - waypoint0.getDistance();
        double slope = deltaV / deltaD;
        double intercept = waypoint0.getVelocity() - slope * waypoint0.getDistance();

        return slope * distance + intercept;
    }

    public static double calculateDistance(double y) {
        return HubLimelightConstants.DISTANCE_CALCULATION_A_COEFFICIENT * Math.pow(y, 2)
                + HubLimelightConstants.DISTANCE_CALCULATION_B_COEFFICIENT * y
                + HubLimelightConstants.DISTANCE_CALCULATION_C_COEFFICIENT;
    }

    /**
     * returns distance, angle, and velocity
     */
    public static double[] calculate(double y) {
        double distance = calculateDistance(y);
        if(SHOOTER_WAYPOINTS.size() == 0)
            return new double[] {0, 0, 0};
        else if(SHOOTER_WAYPOINTS.size() == 1)
            return new double[] {
                    SHOOTER_WAYPOINTS.get(0).getDistance(),
                    SHOOTER_WAYPOINTS.get(0).getAngle(),
                    SHOOTER_WAYPOINTS.get(0).getVelocity()
            };

        // If the distance is smaller than all the points,
        // make a slope using the first two points.
        ShooterWaypoint waypoint0 = SHOOTER_WAYPOINTS.get(0);
        ShooterWaypoint waypoint1 = SHOOTER_WAYPOINTS.get(1);

        // In ShooterZone we sort the shooter waypoints,
        // so we can use it as a sorted array from closest to farthest.
        for(int i = 1; SHOOTER_WAYPOINTS.get(i).getDistance() < distance; i++) {
            // If the distance is bigger than only the max point,
            // make a slope using the last two points.
            if(i + 1 >= SHOOTER_WAYPOINTS.size()) {
                waypoint0 = SHOOTER_WAYPOINTS.get(SHOOTER_WAYPOINTS.size() - 2);
                waypoint1 = SHOOTER_WAYPOINTS.get(SHOOTER_WAYPOINTS.size() - 1);
            } else {
                waypoint0 = SHOOTER_WAYPOINTS.get(i);
                waypoint1 = SHOOTER_WAYPOINTS.get(i + 1);
            }
        }

        double deltaD = waypoint1.getDistance() - waypoint0.getDistance();
        double deltaA = waypoint1.getAngle() - waypoint0.getAngle();
        double deltaV = waypoint1.getVelocity() - waypoint0.getVelocity();
        double vSlope = deltaV / deltaD;
        double aSlope = deltaA / deltaD;
        double vIntercept = waypoint0.getVelocity() - vSlope * waypoint0.getDistance();
        double aIntercept = waypoint0.getAngle() - aSlope * waypoint0.getDistance();

        return new double[] {
                distance,
                vSlope * distance + vIntercept,
                aSlope * distance + aIntercept
        };
    }
}
