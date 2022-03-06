package frc.robot.subsystems.shooter;

public class ShooterWaypoint implements Comparable<ShooterWaypoint> {
    private double distance;
    private double velocity;

    /**
     * Represents a single point at which the shooter has been tested.
     *
     * @param distance The distance of the front of the robot from the reflectors on the hub.
     * @param velocity The ideal velocity to shoot the ball at.
     */
    public ShooterWaypoint(double distance, double velocity) {
        this.distance = distance;
        this.velocity = velocity;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    /**
     * Allows an ArrayList of waypoints to be sorted from closest to farthest.
     */
    @Override
    public int compareTo(ShooterWaypoint waypoint) {
        return (int) (this.distance - waypoint.distance);
    }
}

