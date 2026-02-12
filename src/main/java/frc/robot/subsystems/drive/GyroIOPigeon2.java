// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.pigeonCanId;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private Rotation2d offset = new Rotation2d();

    public GyroIOPigeon2() {
        pigeon.reset();

        yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = OdometryThread.getInstance().registerSignal(() -> pigeon.getYaw().getValueAsDouble());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble()).minus(offset);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());

        inputs.odometryYawTimestams = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public Rotation2d getGyroRotation() {
        return pigeon.getRotation2d().minus(offset);
    }

    @Override
    public AngularVelocity getGyroAngularVelocity() { // TODO verify it return the correct axis
        return AngularVelocity.ofBaseUnits(
                Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble()),
                RadiansPerSecond);
    }

    @Override
    public void reset() {
        reset(new Rotation2d());
    }

    @Override
    public void reset(Rotation2d to) {
        offset = pigeon.getRotation2d().minus(to);
    }
}