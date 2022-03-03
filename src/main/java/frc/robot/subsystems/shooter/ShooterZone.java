package frc.robot.subsystems.shooter;

import com.google.gson.annotations.SerializedName;

import java.util.ArrayList;
import java.util.Collections;

public class ShooterZone {
    @SerializedName(value = "pitcherAngle")
    private double pitcherAngle;
    @SerializedName(value = "waypoints")
    private ArrayList<ShooterWaypoint> waypoints;

    public ShooterZone() {
        waypoints = new ArrayList<>();
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
}
