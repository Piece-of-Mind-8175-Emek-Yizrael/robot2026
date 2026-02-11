package frc.robot.util.BallisticCalculator;

import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.stream.IntStream;

public class BallisticCalculator {

    public enum BallisticCalculatorMode {
        FAST(0.04,0.01),
        ACCURATE(0.02,0.005);

        private double coarseDt, fineDt;

        public double getCoarseDt() {
            return coarseDt;
        }

        public double getFineDt() {
            return fineDt;
        }

        BallisticCalculatorMode(double coarseDt, double fineDt) {
            this.coarseDt = coarseDt;
            this.fineDt = fineDt;
        }
    }

    private static double[] LinSpell(double start, double end, int num) {
        if (num <= 0) return new double[0];
        double[] out = new double[num];
        if (num == 1) {
            out[0] = start;
            return out;
        }
        double step = (end - start) / (num - 1);
        for (int i = 0; i < num; i++) {
            out[i] = start + step * i;
        }
        return out;
    }

    static double minArrival = -90.0;
    static double maxArrival = -40.0;

    static double[] speeds = LinSpell(8.0, 12.0, 20);
    static double[] launchAngles = LinSpell(40.0, 85.0, 45);

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget, double minArrival, double maxArrival, BallisticCalculatorMode mode) {
        return findConstrainedLaunch(
                shooterToTarget.getX(),
                shooterToTarget.getY(),
                0.21,
                0.11,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                BallisticCalculatorMode.ACCURATE
        );
    }

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget, double minArrival, double maxArrival) {
        return findConstrainedLaunch(
                shooterToTarget.getX(),
                shooterToTarget.getY(),
                0.21,
                0.11,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                BallisticCalculatorMode.ACCURATE
        );
    }

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget) {
        return findConstrainedLaunch(
                shooterToTarget.getX(),
                shooterToTarget.getY(),
                0.21,
                0.11,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                BallisticCalculatorMode.ACCURATE
        );
    }

    public static List<BallisticCalculatorResult> calculateForFuel(Translation3d shooterToTarget, BallisticCalculatorMode mode) {
        return findConstrainedLaunch(
                shooterToTarget.getX(),
                shooterToTarget.getY(),
                0.21,
                0.11,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                mode
        );
    }

    private static List<BallisticCalculatorResult> findConstrainedLaunch(
            double targetX, double targetY,
            double weightKg, double radiusM,
            double[] vRange, double[] angleRange,
            double a1deg, double a2deg,
            BallisticCalculatorMode mode
    ) {
        if (weightKg <= 0 || radiusM <= 0) {
            throw new IllegalArgumentException("`weight_kg` and `radius_m` must be positive.");
        }
        if (vRange == null || vRange.length == 0 || angleRange == null || angleRange.length == 0) {
            throw new IllegalArgumentException("`v_range` and `angle_range` must be non-empty.");
        }

        final double g = 9.81;
        final double rho = 1.225;
        final double Cd = 0.48;
        final double area = Math.PI * radiusM * radiusM;
        final double dragConst = 0.5 * rho * Cd * area;
        final double eps = 1e-8;
        final double errThresholdSq = 0.02 * 0.02;

        double aLow = Math.min(a1deg, a2deg);
        double aHigh = Math.max(a1deg, a2deg);

        final int nV = vRange.length;
        final int nA = angleRange.length;
        final int total = nV * nA;

        // precompute trig and flattened initial velocity components
        double[] cosA = new double[nA];
        double[] sinA = new double[nA];
        for (int i = 0; i < nA; i++) {
            double rad = Math.toRadians(angleRange[i]);
            cosA[i] = Math.cos(rad);
            sinA[i] = Math.sin(rad);
        }
        double[] vx0 = new double[total];
        double[] vy0 = new double[total];
        for (int iv = 0; iv < nV; iv++) {
            double v0 = vRange[iv];
            int base = iv * nA;
            for (int ia = 0; ia < nA; ia++) {
                vx0[base + ia] = v0 * cosA[ia];
                vy0[base + ia] = v0 * sinA[ia];
            }
        }

        // Concurrent collection for results (safe from parallel workers)
        ConcurrentLinkedQueue<BallisticCalculatorResult> ballisticCalculatorResults = new ConcurrentLinkedQueue<>();

        // simulation parameters: coarse then fine
        final double coarseDt = mode.getCoarseDt();
        final int coarseMaxSteps = 800; // coarse sweep
        final double coarseThresholdSq = 0.06 * 0.06; // if coarse best below this, refine
        final double fineDt = mode.getFineDt();
        final int fineMaxSteps = 2000; // refinement window (starting from saved coarse state)
        final double xMargin = 0.5;
        final double yFloor = -1.0;

        IntStream.range(0, total).parallel().forEach(idx -> {
            double initVx = vx0[idx];
            double initVy = vy0[idx];
            int iv = idx / nA;
            int ia = idx % nA;
            double v0 = vRange[iv];
            double launchDeg = angleRange[ia];

            // Quick bounding: if initial vx is tiny and targetX far, skip
            if (Math.abs(initVx) < eps && Math.abs(targetX) > 1.0) return;

            // ---- coarse pass ----
            double vx = initVx;
            double vy = initVy;
            double x = 0.0;
            double y = 0.0;
            double bestErrSqCoarse = Double.POSITIVE_INFINITY;
            double savedVx = vx, savedVy = vy, savedX = x, savedY = y;

            for (int step = 0; step < coarseMaxSteps; step++) {
                double v2 = vx * vx + vy * vy;
                double v = Math.sqrt(v2);
                double drag = dragConst * v2;

                double ax = (v > eps) ? -(drag * (vx / v)) / weightKg : 0.0;
                double ay = (v > eps) ? -g - (drag * (vy / v)) / weightKg : -g;

                vx += ax * coarseDt;
                vy += ay * coarseDt;
                x += vx * coarseDt;
                y += vy * coarseDt;

                double dx = x - targetX;
                double dy = y - targetY;
                double errSq = dx * dx + dy * dy;
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
            double bestErrSq = Double.POSITIVE_INFINITY;
            double arrivalAngleDeg = Double.NaN;

            for (int step = 0; step < fineMaxSteps; step++) {
                double v2 = vx * vx + vy * vy;
                double v = Math.sqrt(v2);
                double drag = dragConst * v2;

                double ax = (v > eps) ? -(drag * (vx / v)) / weightKg : 0.0;
                double ay = (v > eps) ? -g - (drag * (vy / v)) / weightKg : -g;

                vx += ax * fineDt;
                vy += ay * fineDt;
                x += vx * fineDt;
                y += vy * fineDt;

                double dx = x - targetX;
                double dy = y - targetY;
                double errSq = dx * dx + dy * dy;
                if (errSq < bestErrSq) {
                    bestErrSq = errSq;
                    arrivalAngleDeg = Math.toDegrees(Math.atan2(vy, vx));
                }

                if (y < yFloor || x > targetX + xMargin || (Math.abs(vx) < eps && Math.abs(vy) < eps)) {
                    break;
                }
            }

            if (!Double.isNaN(arrivalAngleDeg) && arrivalAngleDeg >= aLow && arrivalAngleDeg <= aHigh && bestErrSq < errThresholdSq) {
                double roundedArrival = Math.round(arrivalAngleDeg * 100.0) / 100.0;
                double bestErr = Math.sqrt(bestErrSq);
                ballisticCalculatorResults.add(new BallisticCalculatorResult(v0, launchDeg, roundedArrival, bestErr));
            }
        });

        // convert concurrent queue to list and return
        List<BallisticCalculatorResult> out = new ArrayList<>(ballisticCalculatorResults);
        return out;
    }
}
