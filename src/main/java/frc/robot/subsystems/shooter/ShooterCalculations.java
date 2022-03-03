package frc.robot.subsystems.shooter;

import frc.robot.constants.RobotConstants.ShooterConstants;

import java.util.ArrayList;

public class ShooterCalculations {
    /**
     * @param distance The distance of the front of the robot from the hub
     * @return The zone the robot is in based on the distance from the hub.
     */
    private static int calculateZone(double distance) {
        for(int i = 0; i < ShooterConstants.ZONE_LIMITS.length; i++) {
            if(distance <= ShooterConstants.ZONE_LIMITS[i])
                return i;
        }
        return ShooterConstants.ZONE_LIMITS.length;
    }

    /**
     * Calculates the optimal angle of the pitcher
     * based on the zone the robot is in.
     *
     * @return target angle of the pitcher
     */
    public static double calculateAngle(double distance) {
        return ShooterConstants.SHOOTER_ZONES[calculateZone(distance)].getPitcherAngle();
    }

    /**
     * Calculates the optimal velocity of the shooter
     * based on the zone the robot is in using a slope between two points.
     *
     * @return target velocity of the shooter
     */
    public static double calculateVelocity(double distance) {
        ShooterZone zone = ShooterConstants.SHOOTER_ZONES[calculateZone(distance)];
        ArrayList<ShooterWaypoint> waypoints = zone.getWaypoints();

        ShooterWaypoint waypoint0 = waypoints.get(0);
        ShooterWaypoint waypoint1 = waypoints.get(0);

        // In ShooterZone we sort the shooter waypoints,
        // so we can use it as a sorted array from closest to farthest.
        for(int i = 0; i < waypoints.size(); i++) {
            if(distance >= waypoints.get(i).getDistance()) {
                waypoint0 = waypoints.get(i);
                // If the distance in bigger than only the max point act as if it is the max point.
                if(i + 1 >= waypoints.size())
                    return waypoint0.getVelocity();
                waypoint1 = waypoints.get(i + 1);
                break;
            }
        }
        // If the distance in smaller than all the points act as if it is the max point.
        if(waypoint1 == waypoint0)
            return waypoint0.getVelocity();

        double deltaV = waypoint1.getVelocity() - waypoint0.getVelocity();
        double deltaD = waypoint1.getDistance() - waypoint0.getDistance();
        double slope = deltaV / deltaD;
        double intercept = waypoint0.getVelocity() - slope * waypoint0.getDistance();

        return slope * distance + intercept;
    }
}
