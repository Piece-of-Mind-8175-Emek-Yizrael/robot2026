package frc.robot.util.BallisticCalculator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.concurrent.locks.Lock;

public class BallisticCalculator {

    public enum BallisticCalculatorMode {
        FAST(0.04f,0.01f),
        ACCURATE(0.02f,0.005f);

        private final float coarseDt, fineDt;

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

    private volatile boolean parametersUpdated = false;
    private volatile boolean isProcessing = false;
    private volatile BallisticCalculatorParameters parameters;
    private volatile BallisticCalculatorResultWithRotation result = null;
    private Thread processingTread = null;
    private volatile Lock resultsLock;

    private static BallisticCalculator instance;

    public static BallisticCalculator getInstance() {
        if (instance == null) {
            instance = new BallisticCalculator();
        }
        return instance;
    }

    public BallisticCalculator() {
        parameters = new BallisticCalculatorParameters(
                0.0f,
                0.0f,
                0.21f,
                0.11f,
                speeds,
                launchAngles,
                minArrival,
                maxArrival,
                new Translation2d(),
                BallisticCalculatorMode.ACCURATE
        );
        processingTread = new Thread(() -> {
            BallisticCalculatorResultWithRotation res = null;
            while(true) {
                while (!parametersUpdated) ;
                isProcessing = true;
                res = findConstrainedLaunchInMotion(parameters);
                parametersUpdated = false;
                isProcessing = false;
                while (resultsLock.tryLock()) ;
                try {
                    result = res;
                } finally {
                    resultsLock.unlock();
                }
            }
        });
        processingTread.run();
    }

    public BallisticCalculatorResultWithRotation getLatestResults() {
        while (resultsLock.tryLock()) ;
        try {
            return result;
        } finally {
            resultsLock.unlock();
        }
    }

    public void updateParameters(Translation3d shooterToTarget, Translation2d targetCentricMovement) {
        if (isProcessing) return;
        parameters = new BallisticCalculatorParameters(
                (float) shooterToTarget.getX(),
                (float) shooterToTarget.getY(),
                parameters.weightKg(),
                parameters.radiusM(),
                parameters.vRange(),
                parameters.angleRange(),
                parameters.a1deg(),
                parameters.a2deg(),
                targetCentricMovement,
                parameters.mode()
        );
        parametersUpdated = true;
    }

    private BallisticCalculatorResultWithRotation findConstrainedLaunchInMotion(
            BallisticCalculatorParameters params
    ) {
        if (params.weightKg() <= 0 || params.radiusM() <= 0) {
            throw new IllegalArgumentException("`weight_kg` and `radius_m` must be positive.");
        }
        if (params.vRange() == null || params.vRange().length == 0 || params.angleRange() == null || params.angleRange().length == 0) {
            throw new IllegalArgumentException("`v_range` and `angle_range` must be non-empty.");
        }
        if (params.targetCentricMovement() == null) {
            throw new IllegalArgumentException("`target_centric_movement` must be non-empty.");
        }

        final float g = 9.81f;
        final float rho = 1.225f;
        final float Cd = 0.48f;
        final float area = (float)Math.PI * params.radiusM() * params.radiusM();
        final float dragConst = 0.5f * rho * Cd * area;
        final float eps = 1e-8f;
        final float errThresholdSq = 0.02f * 0.02f;

        float aLow = Math.min(params.a1deg(), params.a2deg());
        float aHigh = Math.max(params.a1deg(), params.a2deg());

        final int nV = params.vRange().length;
        final int nA = params.angleRange().length;
        final int total = nV * nA;

        // precompute trig and flattened initial velocity components
        float[] cosA = new float[nA];
        float[] sinA = new float[nA];
        for (int i = 0; i < nA; i++) {
            float rad = (float)Math.toRadians(params.angleRange()[i]);
            cosA[i] = (float)Math.cos(rad);
            sinA[i] = (float)Math.sin(rad);
        }
        float[] vx0 = new float[total];
        float[] vy0 = new float[total];

        // TODO: check whether the X and Y of the targetCentricMovement are applied at the correct order
        for (int iv = nV - 1; iv >= 0; iv--) {
            float v0 = params.vRange()[iv];
            int base = iv * nA;
            for (int ia = 0; ia < nA; ia++) {
                vx0[base + ia] = v0 * cosA[ia] + (float) params.targetCentricMovement().getX();
                vy0[base + ia] = v0 * sinA[ia];
            }
        }

        // simulation parameters: coarse then fine
        final float coarseDt = params.mode().getCoarseDt();
        final int coarseMaxSteps = 800; // coarse sweep
        final float coarseThresholdSq = 0.06f * 0.06f; // if coarse best below this, refine
        final float fineDt = params.mode().getFineDt();
        final int fineMaxSteps = 2000; // refinement window (starting from saved coarse state)
        final float xMargin = 0.5f;
        final float yFloor = -1.0f;

        for (int idx = 0; idx < vx0.length; idx++) {
            float initVx = vx0[idx];
            float initVy = vy0[idx];
            int iv = idx / nA;
            int ia = idx % nA;
            float v0 = params.vRange()[iv];
            float launchDeg = params.angleRange()[ia];

            // Quick bounding: if initial vx is tiny and targetX far, skip
            if (Math.abs(initVx) < eps && Math.abs(params.targetX()) > 1.0) continue;

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

                float ax = (v > eps) ? -(drag * (vx / v)) / params.weightKg() : 0.0f;
                float ay = (v > eps) ? -g - (drag * (vy / v)) / params.weightKg() : -g;

                vx += ax * coarseDt;
                vy += ay * coarseDt;
                x += vx * coarseDt;
                y += vy * coarseDt;


                float dx = x - params.targetX();
                float dy = y - params.targetY();
                float errSq = dx * dx + dy * dy;
                if (errSq < bestErrSqCoarse) {
                    bestErrSqCoarse = errSq;
                    savedVx = vx;
                    savedVy = vy;
                    savedX = x;
                    savedY = y;
                }
                if (y < yFloor || x > params.targetX() + xMargin || (Math.abs(vx) < eps && Math.abs(vy) < eps)) break;
            }

            // If coarse pass indicates not promising, skip
            if (bestErrSqCoarse > coarseThresholdSq) continue;

            // ---- fine pass: start from saved coarse state and refine ----
            vx = savedVx;
            vy = savedVy;

            x = savedX;
            y = savedY;

            float bestErrSq = Float.POSITIVE_INFINITY;
            float arrivalAngleDeg = Float.NaN;
            
            double vz = params.targetCentricMovement().getY();
            double ez = 0;

            for (int step = 0; step < fineMaxSteps; step++) {
                float v2 = vx * vx + vy * vy;
                float v = (float)Math.sqrt(v2);
                float drag = dragConst * v2;

                float ax = (v > eps) ? -(drag * (vx / v)) / params.weightKg() : 0.0f;
                float ay = (v > eps) ? -g - (drag * (vy / v)) / params.weightKg() : -g;

                vx += ax * fineDt;
                vy += ay * fineDt;
                x += vx * fineDt;
                y += vy * fineDt;

                double dragZ = dragConst * vz * vz;
                double az = -dragZ / params.weightKg();
                vz += az * fineDt;
                ez += vz * fineDt;

                float dx = x - params.targetX();
                float dy = y - params.targetY();
                float errSq = dx * dx + dy * dy;
                if (errSq < bestErrSq) {
                    bestErrSq = errSq;
                    arrivalAngleDeg = (float)Math.toDegrees(Math.atan2(vy, vx));
                }

                if (y < yFloor || x > params.targetX() + xMargin || (Math.abs(vx) < eps && Math.abs(vy) < eps)) {
                    break;
                }
            }

            if (!Double.isNaN(arrivalAngleDeg) && arrivalAngleDeg >= aLow && arrivalAngleDeg <= aHigh && bestErrSq < errThresholdSq) {
                // find the required shooter rotation for the provided time of flight
                double dRotation = Math.toDegrees(Math.atan2(ez, params.targetX()));

                float roundedArrival = Math.round(arrivalAngleDeg * 100.0f) / 100.0f;
                float bestErr = (float)Math.sqrt(bestErrSq);
                return new BallisticCalculatorResultWithRotation(v0, launchDeg, roundedArrival, bestErr, dRotation);
            }
        }

        // convert concurrent queue to list and return
        return null;
    }
}
