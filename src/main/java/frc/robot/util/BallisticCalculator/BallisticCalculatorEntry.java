package frc.robot.util.BallisticCalculator;

import com.fasterxml.jackson.annotation.JsonGetter;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;

@JsonPropertyOrder({"distance", "velocity", "angle"})
public class BallisticCalculatorEntry {
    double distanceFromHubX;
    double shooterAngle;
    double projectileVelocity;

    public BallisticCalculatorEntry(double distanceFromHubX, double shooterAngle, double projectileVelocity) {
        this.distanceFromHubX = distanceFromHubX;
        this.shooterAngle = shooterAngle;
        this.projectileVelocity = projectileVelocity;
    }

    @JsonGetter("distance")
    public double getDistanceFromHubX() {
        return distanceFromHubX;
    }

    public void setDistanceFromHubX(double distanceFromHubX) {
        this.distanceFromHubX = distanceFromHubX;
    }

    @JsonGetter("angle")
    public double getShooterAngle() {
        return shooterAngle;
    }

    public void setShooterAngle(double shooterAngle) {
        this.shooterAngle = shooterAngle;
    }

    @JsonGetter("velocity")
    public double getProjectileVelocity() {
        return projectileVelocity;
    }

    public void setProjectileVelocity(double projectileVelocity) {
        this.projectileVelocity = projectileVelocity;
    }

    @Override
    public String toString() {
        return "distance: " + distanceFromHubX + "[m], angle: " + shooterAngle + "[deg], velocity: " + projectileVelocity + "[m/s]";
    }
}
