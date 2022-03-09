package frc.robot.subsystems.shooter;

import java.util.ArrayList;

public class ShootingCalculations {
    //TODO: Set zone limits and coefs
    private static final double[] ZONE_LIMITS = new double[] {1};
    private static final ShooterZone[] SHOOTER_ZONES = new ShooterZone[] {
            new ShooterZone(
                    15, new ShooterWaypoint(2.16, 3177), new ShooterWaypoint(2.905, 3650),
                    new ShooterWaypoint(1.77, 2900), new ShooterWaypoint(4.4, 4200)),
            new ShooterZone(15, new ShooterWaypoint(2.16, 3177), new ShooterWaypoint(2.905, 3650),
                    new ShooterWaypoint(1.77, 2900), new ShooterWaypoint(4.4, 4200))
    };
    //            new ShooterZone(5, new ShooterWaypoint(0.67, 2777), new ShooterWaypoint(1.31, 3000)),

    /**
     * @param distance The distance of the front of the robot from the hub
     * @return The zone the robot is in based on the distance from the hub.
     */
    private static int calculateZone(double distance) {
        for(int i = 0; i < ZONE_LIMITS.length; i++) {
            if(distance <= ZONE_LIMITS[i])
                return i;
        }
        return ZONE_LIMITS.length - 1;
    }

    /**
     * Calculates the optimal angle of the pitcher
     * based on the zone the robot is in.
     *
     * @return target angle of the pitcher
     */
    public static double calculateAngle(double distance) {
        return SHOOTER_ZONES[calculateZone(distance)].getPitcherAngle();
    }

    /**
     * Calculates the optimal velocity of the shooter
     * based on the zone the robot is in using a slope between two points.
     *
     * @return target velocity of the shooter
     */
    public static double calculateVelocity(double distance) {
        ShooterZone zone = SHOOTER_ZONES[calculateZone(distance)];
        ArrayList<ShooterWaypoint> waypoints = zone.getWaypoints();

        if(waypoints.size() == 0)
            return 0;
        else if(waypoints.size() == 1)
            return waypoints.get(0).getVelocity();

        // If the distance is smaller than all the points,
        // make a slope using the first two points.
        ShooterWaypoint waypoint0 = waypoints.get(0);
        ShooterWaypoint waypoint1 = waypoints.get(1);

        // In ShooterZone we sort the shooter waypoints,
        // so we can use it as a sorted array from closest to farthest.
        for(int i = 0; i < waypoints.size(); i++) {
            if(distance >= waypoints.get(i).getDistance()) {
                waypoint0 = waypoints.get(i);
                // If the distance is bigger than only the max point,
                // make a slope using the last two points.
                if(i + 1 >= waypoints.size()) {
                    waypoint0 = waypoints.get(waypoints.size() - 2);
                    waypoint1 = waypoints.get(waypoints.size() - 1);
                }
                waypoint1 = waypoints.get(i + 1);
                break;
            }
        }

        double deltaV = waypoint1.getVelocity() - waypoint0.getVelocity();
        double deltaD = waypoint1.getDistance() - waypoint0.getDistance();
        double slope = deltaV / deltaD;
        double intercept = waypoint0.getVelocity() - slope * waypoint0.getDistance();

        return slope * distance + intercept;
    }
}
