package frc.robot.util.BallisticCalculator;

import com.fasterxml.jackson.annotation.JsonGetter;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;
import com.fasterxml.jackson.core.JacksonException;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;

import java.io.IOException;

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

class BallisticTableDeserializer extends StdDeserializer<BallisticCalculatorEntry> {

    public BallisticTableDeserializer() {
        this(null);
    }

    protected BallisticTableDeserializer(Class<?> vc) {
        super(vc);
    }

    // TODO: implement the deserialization function
    @Override
    public BallisticCalculatorEntry deserialize(JsonParser jsonParser, DeserializationContext deserializationContext) throws IOException, JacksonException {
        JsonNode node = jsonParser.getCodec().readTree(jsonParser);
        return null;
    }
}