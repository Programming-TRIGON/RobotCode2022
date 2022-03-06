package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class ShooterZone implements Sendable {
    private double pitcherAngle;
    private ArrayList<ShooterWaypoint> waypoints;

    private double tempWaypointDistance;
    private double tempWaypointVelocity;

    public ShooterZone() {
        waypoints = new ArrayList<>();
    }

    public ShooterZone(double pitcherAngle, ShooterWaypoint... waypoints) {
        this.pitcherAngle = pitcherAngle;
        this.waypoints = new ArrayList<>(Arrays.asList(waypoints));
        Collections.sort(this.waypoints);
    }

    public double getPitcherAngle() {
        return pitcherAngle;
    }

    public void setPitcherAngle(double pitcherAngle) {
        this.pitcherAngle = pitcherAngle;
    }

    public void addShooterWaypoints(ShooterWaypoint waypoint) {
        waypoints.add(waypoint);
        Collections.sort(waypoints);
    }

    public void addShooterWaypoints(double distance, double velocity) {
        waypoints.add(new ShooterWaypoint(distance, velocity));
    }

    public ArrayList<ShooterWaypoint> getWaypoints() {
        return waypoints;
    }

    public void setWaypoints(ArrayList<ShooterWaypoint> waypoints) {
        this.waypoints = waypoints;
    }

    public void setTempWaypointDistance(double tempWaypointDistance) {
        this.tempWaypointDistance = tempWaypointDistance;
    }

    public void setTempWaypointVelocity(double tempWaypointVelocity) {
        this.tempWaypointVelocity = tempWaypointVelocity;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty(
                "waypoint distance", () -> tempWaypointDistance, (distance) -> setTempWaypointDistance(distance));
        builder.addDoubleProperty(
                "waypoint velocity", () -> tempWaypointVelocity, (velocity) -> setTempWaypointVelocity(velocity));
        builder.addBooleanProperty(
                "add waypoint", () -> false, (x) -> addShooterWaypoints(tempWaypointDistance, tempWaypointVelocity));
    }
}
