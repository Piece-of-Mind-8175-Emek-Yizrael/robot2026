package frc.robot.util.BallisticCalculator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public record BallisticCalculatorParameters(float targetX, float targetY,
                                            float weightKg, float radiusM,
                                            float[] vRange, float[] angleRange,
                                            float a1deg, float a2deg,
                                            Translation2d targetCentricMovement,
                                            BallisticCalculator.BallisticCalculatorMode mode) {
}
