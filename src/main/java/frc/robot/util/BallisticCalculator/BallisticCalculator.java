package frc.robot.util.BallisticCalculator;

import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.IntStream;

public class BallisticCalculator {

    public enum BallisticCalculatorMode {
        FAST(0.04f,0.01f),
        ACCURATE(0.02f,0.005f);

        private float coarseDt, fineDt;

        public float getCoarseDt() {
            return coarseDt;
        }

        public float getFineDt() {
            return fineDt;
        }

        BallisticCalculatorMode(float coarseDt, float fineDt) {
            this.coarseDt = coarseDt;
            this.fineDt = fineDt;
        }
    }

    private static float[] LinSpell(float start, float end, int num) {
        if (num <= 0) return new float[0];
        float[] out = new float[num];
        if (num == 1) {
            out[0] = start;
            return out;
        }
        float step = (end - start) / (num - 1);
        for (int i = 0; i < num; i++) {
            out[i] = start + step * i;
        }
        return out;
    }

    static float minArrival = -90.0f;
    static float maxArrival = -40.0f;

    static float[] speeds = LinSpell(8.0f, 12.0f, 20);
    static float[] launchAngles = LinSpell(40.0f, 85.0f, 45);

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget, float minArrival, float maxArrival, BallisticCalculatorMode mode) {
        return findConstrainedLaunch(
                (float)shooterToTarget.getX(),
                (float)shooterToTarget.getY(),
                0.21f,
                0.11f,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                BallisticCalculatorMode.ACCURATE
        );
    }

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget, float minArrival, float maxArrival) {
        return findConstrainedLaunch(
                (float)shooterToTarget.getX(),
                (float)shooterToTarget.getY(),
                0.21f,
                0.11f,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                BallisticCalculatorMode.ACCURATE
        );
    }

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget) {
        return findConstrainedLaunch(
                (float)shooterToTarget.getX(),
                (float)shooterToTarget.getY(),
                0.21f,
                0.11f,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                BallisticCalculatorMode.ACCURATE
        );
    }

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget, BallisticCalculatorMode mode) {
        return findConstrainedLaunch(
                (float)shooterToTarget.getX(),
                (float)shooterToTarget.getY(),
                0.21f,
                0.11f,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                mode
        );
    }

    private static List<BallisticCalculatorResult> findConstrainedLaunch(
            float targetX, float targetY,
            float weightKg, float radiusM,
            float[] vRange, float[] angleRange,
            float a1deg, float a2deg,
            BallisticCalculatorMode mode
    ) {
        if (weightKg <= 0 || radiusM <= 0) {
            throw new IllegalArgumentException("`weight_kg` and `radius_m` must be positive.");
        }
        if (vRange == null || vRange.length == 0 || angleRange == null || angleRange.length == 0) {
            throw new IllegalArgumentException("`v_range` and `angle_range` must be non-empty.");
        }

        final float g = 9.81f;
        final float rho = 1.225f;
        final float Cd = 0.48f;
        final float area = (float)Math.PI * radiusM * radiusM;
        final float dragConst = 0.5f * rho * Cd * area;
        final float eps = 1e-8f;
        final float errThresholdSq = 0.02f * 0.02f;

        float aLow = Math.min(a1deg, a2deg);
        float aHigh = Math.max(a1deg, a2deg);

        final int nV = vRange.length;
        final int nA = angleRange.length;
        final int total = nV * nA;

        // precompute trig and flattened initial velocity components
        float[] cosA = new float[nA];
        float[] sinA = new float[nA];
        for (int i = 0; i < nA; i++) {
            float rad = (float)Math.toRadians(angleRange[i]);
            cosA[i] = (float)Math.cos(rad);
            sinA[i] = (float)Math.sin(rad);
        }
        float[] vx0 = new float[total];
        float[] vy0 = new float[total];
        for (int iv = 0; iv < nV; iv++) {
            float v0 = vRange[iv];
            int base = iv * nA;
            for (int ia = 0; ia < nA; ia++) {
                vx0[base + ia] = v0 * cosA[ia];
                vy0[base + ia] = v0 * sinA[ia];
            }
        }

        // Concurrent collection for results (safe from parallel workers)
        ConcurrentLinkedQueue<BallisticCalculatorResult> ballisticCalculatorResults = new ConcurrentLinkedQueue<>();

        // simulation parameters: coarse then fine
        final float coarseDt = mode.getCoarseDt();
        final int coarseMaxSteps = 800; // coarse sweep
        final float coarseThresholdSq = 0.06f * 0.06f; // if coarse best below this, refine
        final float fineDt = mode.getFineDt();
        final int fineMaxSteps = 2000; // refinement window (starting from saved coarse state)
        final float xMargin = 0.5f;
        final float yFloor = -1.0f;

        IntStream.range(0, total).parallel().forEach(idx -> {
            float initVx = vx0[idx];
            float initVy = vy0[idx];
            int iv = idx / nA;
            int ia = idx % nA;
            float v0 = vRange[iv];
            float launchDeg = angleRange[ia];

            // Quick bounding: if initial vx is tiny and targetX far, skip
            if (Math.abs(initVx) < eps && Math.abs(targetX) > 1.0) return;

            // ---- coarse pass ----
            float vx = initVx;
            float vy = initVy;
            float x = 0.0f;
            float y = 0.0f;
            float bestErrSqCoarse = Float.POSITIVE_INFINITY;
            float savedVx = vx, savedVy = vy, savedX = x, savedY = y;

            for (int step = 0; step < coarseMaxSteps; step++) {
                float v2 = vx * vx + vy * vy;
                float v = (float)Math.sqrt(v2);
                float drag = dragConst * v2;

                float ax = (v > eps) ? -(drag * (vx / v)) / weightKg : 0.0f;
                float ay = (v > eps) ? -g - (drag * (vy / v)) / weightKg : -g;

                vx += ax * coarseDt;
                vy += ay * coarseDt;
                x += vx * coarseDt;
                y += vy * coarseDt;

                float dx = x - targetX;
                float dy = y - targetY;
                float errSq = dx * dx + dy * dy;
                if (errSq < bestErrSqCoarse) {
                    bestErrSqCoarse = errSq;
                    savedVx = vx;
                    savedVy = vy;
                    savedX = x;
                    savedY = y;
                }
                if (y < yFloor || x > targetX + xMargin || (Math.abs(vx) < eps && Math.abs(vy) < eps)) break;
            }

            // If coarse pass indicates not promising, skip
            if (bestErrSqCoarse > coarseThresholdSq) return;

            // ---- fine pass: start from saved coarse state and refine ----
            vx = savedVx;
            vy = savedVy;
            x = savedX;
            y = savedY;
            float bestErrSq = Float.POSITIVE_INFINITY;
            float arrivalAngleDeg = Float.NaN;

            for (int step = 0; step < fineMaxSteps; step++) {
                float v2 = vx * vx + vy * vy;
                float v = (float)Math.sqrt(v2);
                float drag = dragConst * v2;

                float ax = (v > eps) ? -(drag * (vx / v)) / weightKg : 0.0f;
                float ay = (v > eps) ? -g - (drag * (vy / v)) / weightKg : -g;

                vx += ax * fineDt;
                vy += ay * fineDt;
                x += vx * fineDt;
                y += vy * fineDt;

                float dx = x - targetX;
                float dy = y - targetY;
                float errSq = dx * dx + dy * dy;
                if (errSq < bestErrSq) {
                    bestErrSq = errSq;
                    arrivalAngleDeg = (float)Math.toDegrees(Math.atan2(vy, vx));
                }

                if (y < yFloor || x > targetX + xMargin || (Math.abs(vx) < eps && Math.abs(vy) < eps)) {
                    break;
                }
            }

            if (!Double.isNaN(arrivalAngleDeg) && arrivalAngleDeg >= aLow && arrivalAngleDeg <= aHigh && bestErrSq < errThresholdSq) {
                float roundedArrival = Math.round(arrivalAngleDeg * 100.0f) / 100.0f;
                float bestErr = (float)Math.sqrt(bestErrSq);
                ballisticCalculatorResults.add(new BallisticCalculatorResult(v0, launchDeg, roundedArrival, bestErr));
            }
        });

        // convert concurrent queue to list and return
        List<BallisticCalculatorResult> out = new ArrayList<>(ballisticCalculatorResults);
        return out;
    }
}
