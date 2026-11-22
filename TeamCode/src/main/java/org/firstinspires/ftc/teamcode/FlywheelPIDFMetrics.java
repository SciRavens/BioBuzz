package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Hardware-decoupled flywheel PIDF math, multi-ball shot recovery tracking,
 * and coordinate pattern-search utilities.
 *
 * Units: FTC {@code DcMotorEx.setVelocity(double)} commands encoder ticks per second.
 * For a bare 6000 RPM motor with 28 ticks/rev, 6000 RPM corresponds to 2800 ticks/sec.
 */
public final class FlywheelPIDFMetrics {

    public static final double MOTOR_MAX_RPM = 6000.0;
    public static final double TICKS_PER_REV = 28.0;
    public static final double F_MAX_VELOCITY = 32767.0;
    public static final double MAX_HEADROOM_FRACTION = 0.85;

    private FlywheelPIDFMetrics() {}

    /** Converts encoder ticks/sec to physical motor RPM using {@link #TICKS_PER_REV}. */
    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    /** Converts physical motor RPM to encoder ticks/sec using {@link #TICKS_PER_REV}. */
    public static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    /**
     * Computes REV Hub velocity feedforward F = 32767 / maxVelocity (ticks/s).
     * Returns 0.0 for non-positive or non-finite inputs.
     */
    public static double computeFeedforwardF(double maxVelocityTicksPerSec) {
        if (!Double.isFinite(maxVelocityTicksPerSec) || maxVelocityTicksPerSec <= 0.0) {
            return 0.0;
        }
        return F_MAX_VELOCITY / maxVelocityTicksPerSec;
    }

    /**
     * Returns the recommended upper target velocity (85% of free-spin max) so the
     * controller retains headroom to recover quickly after ball impacts.
     */
    public static double recommendedMaxTarget(double maxVelocityTicksPerSec) {
        if (!Double.isFinite(maxVelocityTicksPerSec) || maxVelocityTicksPerSec <= 0.0) {
            return 0.0;
        }
        return maxVelocityTicksPerSec * MAX_HEADROOM_FRACTION;
    }

    /**
     * Computes a scalar cost (lower is better) for an auto-tune trial.
     * Penalizes integral tracking error, overshoot, tail jitter, and steady-state offset.
     */
    public static double computeTrialCost(
            double targetVelocity,
            double iae,
            double windowSeconds,
            double peakVelocity,
            double tailStdDev,
            double finalSteadyError) {
        if (targetVelocity <= 0.0 || windowSeconds <= 0.0) {
            return 1e9;
        }
        double trackingPct = 100.0 * iae / (targetVelocity * windowSeconds);
        double overshootPct = 100.0 * Math.max(0.0, peakVelocity - targetVelocity) / targetVelocity;
        double jitterPct = 100.0 * Math.max(0.0, tailStdDev) / targetVelocity;
        double steadyOffsetPct = 100.0 * Math.abs(finalSteadyError) / targetVelocity;
        return trackingPct + 1.5 * overshootPct + 0.5 * jitterPct + 2.0 * steadyOffsetPct;
    }

    /** Metrics captured for a single pollen passing through the flywheel. */
    public static final class ShotRecord {
        public final int shotNumber;
        public final double preShotVelocity;
        public final double minVelocity;
        public final double dipDepth;
        public final double recoveryTimeMs; // -1 if flywheel never recovered before burst ended
        public final double peakOvershoot;
        public final double settledVelocity;
        public final double steadyStateOffset; // target - settledVelocity (+ means drooping below target)

        public ShotRecord(
                int shotNumber,
                double preShotVelocity,
                double minVelocity,
                double dipDepth,
                double recoveryTimeMs,
                double peakOvershoot,
                double settledVelocity,
                double steadyStateOffset) {
            this.shotNumber = shotNumber;
            this.preShotVelocity = preShotVelocity;
            this.minVelocity = minVelocity;
            this.dipDepth = dipDepth;
            this.recoveryTimeMs = recoveryTimeMs;
            this.peakOvershoot = peakOvershoot;
            this.settledVelocity = settledVelocity;
            this.steadyStateOffset = steadyStateOffset;
        }

        public boolean recovered() {
            return recoveryTimeMs >= 0.0;
        }
    }

    /**
     * Tracks per-ball velocity dips, recovery times, overshoot, and post-shot
     * steady-state offset across a multi-ball shooting burst.
     */
    public static final class ShotTracker {
        public enum State {
            WAITING_FOR_TARGET,
            ARMED,
            IN_DIP,
            RECOVERED_SETTLING
        }

        private static final double DEFAULT_TOLERANCE_FRAC = 0.025;
        private static final double DEFAULT_MIN_TOLERANCE = 25.0;
        private static final double DEFAULT_DIP_TRIGGER_FRAC = 0.04;
        private static final double DEFAULT_MIN_DIP_TRIGGER = 40.0;
        private static final double DEFAULT_SETTLE_WINDOW_MS = 60.0;

        private double targetVelocity;
        private double tolerance;
        private double dipTrigger;
        private final double settleWindowMs;

        private State state = State.WAITING_FOR_TARGET;
        private final List<ShotRecord> shots = new ArrayList<>();

        private double preShotVel;
        private double dipStartMs;
        private double minVel;
        private double peakVel;
        private double settleStartMs = -1.0;
        private double settledSum;
        private int settledCount;

        public ShotTracker(double targetVelocity) {
            this(
                    targetVelocity,
                    Math.max(DEFAULT_MIN_TOLERANCE, Math.abs(targetVelocity) * DEFAULT_TOLERANCE_FRAC),
                    Math.max(DEFAULT_MIN_DIP_TRIGGER, Math.abs(targetVelocity) * DEFAULT_DIP_TRIGGER_FRAC),
                    DEFAULT_SETTLE_WINDOW_MS);
        }

        public ShotTracker(
                double targetVelocity,
                double toleranceTicksPerSec,
                double dipTriggerTicksPerSec,
                double settleWindowMs) {
            this.settleWindowMs = Math.max(0.0, settleWindowMs);
            configureBands(targetVelocity, toleranceTicksPerSec, dipTriggerTicksPerSec);
            reset(targetVelocity);
        }

        private void configureBands(double target, double tol, double dipTrig) {
            this.targetVelocity = Math.max(0.0, target);
            this.tolerance = Math.max(1.0, tol);
            this.dipTrigger = Math.max(this.tolerance, dipTrig);
        }

        public void reset(double newTargetVelocity) {
            configureBands(
                    newTargetVelocity,
                    Math.max(DEFAULT_MIN_TOLERANCE, Math.abs(newTargetVelocity) * DEFAULT_TOLERANCE_FRAC),
                    Math.max(DEFAULT_MIN_DIP_TRIGGER, Math.abs(newTargetVelocity) * DEFAULT_DIP_TRIGGER_FRAC));
            resetPreservingBands();
        }

        public void resetPreservingBands() {
            shots.clear();
            state = State.WAITING_FOR_TARGET;
            preShotVel = targetVelocity;
            dipStartMs = 0.0;
            minVel = targetVelocity;
            peakVel = targetVelocity;
            settleStartMs = -1.0;
            settledSum = 0.0;
            settledCount = 0;
        }

        public State getState() {
            return state;
        }

        public boolean isArmedOrActive() {
            return state != State.WAITING_FOR_TARGET;
        }

        /**
         * Feeds one telemetry sample into the shot tracker.
         *
         * @param timeMs   monotonic timestamp in milliseconds
         * @param velocity current flywheel velocity (ticks/sec)
         * @param feeding  true while the intake/gate is feeding balls
         */
        public void update(double timeMs, double velocity, boolean feeding) {
            if (targetVelocity <= 0.0 || !Double.isFinite(velocity)) {
                return;
            }
            boolean inBand = Math.abs(velocity - targetVelocity) <= tolerance;

            switch (state) {
                case WAITING_FOR_TARGET:
                    if (inBand) {
                        if (settleStartMs < 0.0) {
                            settleStartMs = timeMs;
                        }
                        if (timeMs - settleStartMs >= settleWindowMs) {
                            state = State.ARMED;
                            preShotVel = velocity;
                            settleStartMs = -1.0;
                        }
                    } else {
                        settleStartMs = -1.0;
                    }
                    break;

                case ARMED:
                    if (feeding && velocity < targetVelocity - dipTrigger) {
                        state = State.IN_DIP;
                        dipStartMs = timeMs;
                        minVel = velocity;
                        peakVel = velocity;
                        settleStartMs = -1.0;
                        settledSum = 0.0;
                        settledCount = 0;
                    } else if (inBand) {
                        preShotVel = velocity;
                    }
                    break;

                case IN_DIP:
                    minVel = Math.min(minVel, velocity);
                    peakVel = Math.max(peakVel, velocity);
                    if (inBand) {
                        state = State.RECOVERED_SETTLING;
                        settleStartMs = timeMs;
                        settledSum = velocity;
                        settledCount = 1;
                    }
                    break;

                case RECOVERED_SETTLING:
                    peakVel = Math.max(peakVel, velocity);
                    if (velocity < targetVelocity - tolerance) {
                        // Dropped back below band before stabilizing -> still recovering from dip
                        state = State.IN_DIP;
                        minVel = Math.min(minVel, velocity);
                        settleStartMs = -1.0;
                        settledSum = 0.0;
                        settledCount = 0;
                    } else if (velocity > targetVelocity + tolerance) {
                        // Overshot above band; reset settle window until damping brings it back
                        settleStartMs = -1.0;
                        settledSum = 0.0;
                        settledCount = 0;
                    } else {
                        if (settleStartMs < 0.0) {
                            settleStartMs = timeMs;
                            settledSum = velocity;
                            settledCount = 1;
                        } else {
                            settledSum += velocity;
                            settledCount++;
                        }
                        if (timeMs - settleStartMs >= settleWindowMs) {
                            double settledVel = settledSum / settledCount;
                            recordShot(settleStartMs - dipStartMs, settledVel);
                            state = State.ARMED;
                            preShotVel = settledVel;
                            settleStartMs = -1.0;
                        }
                    }
                    break;
            }
        }

        /**
         * Finalizes any in-progress dip when a burst ends so unrecovered or
         * steady-state-offset shots are still reported.
         */
        public void finishBurst(double timeMs, double currentVelocity) {
            if (state == State.IN_DIP || state == State.RECOVERED_SETTLING) {
                double settled = settledCount > 0 ? settledSum / settledCount : currentVelocity;
                double recovMs = (state == State.RECOVERED_SETTLING && settleStartMs >= 0.0)
                        ? (settleStartMs - dipStartMs)
                        : -1.0;
                recordShot(recovMs, settled);
                state = State.ARMED;
                settleStartMs = -1.0;
            }
        }

        private void recordShot(double recoveryTimeMs, double settledVel) {
            double dipDepth = Math.max(0.0, targetVelocity - minVel);
            double overshoot = Math.max(0.0, peakVel - targetVelocity);
            double offset = targetVelocity - settledVel;
            shots.add(new ShotRecord(
                    shots.size() + 1,
                    preShotVel,
                    minVel,
                    dipDepth,
                    recoveryTimeMs,
                    overshoot,
                    settledVel,
                    offset));
        }

        public List<ShotRecord> getShots() {
            return Collections.unmodifiableList(new ArrayList<>(shots));
        }

        public int getShotCount() {
            return shots.size();
        }

        public double getMaxDipDepth() {
            double max = 0.0;
            for (ShotRecord s : shots) {
                max = Math.max(max, s.dipDepth);
            }
            return max;
        }

        public double getWorstRecoveryMs() {
            double max = 0.0;
            for (ShotRecord s : shots) {
                if (s.recoveryTimeMs >= 0.0) {
                    max = Math.max(max, s.recoveryTimeMs);
                }
            }
            return max;
        }

        public double getMaxAbsSteadyStateOffset() {
            double max = 0.0;
            for (ShotRecord s : shots) {
                max = Math.max(max, Math.abs(s.steadyStateOffset));
            }
            return max;
        }

        public int getUnrecoveredCount() {
            int count = 0;
            for (ShotRecord s : shots) {
                if (!s.recovered()) {
                    count++;
                }
            }
            return count;
        }
    }

    /**
     * Coordinate pattern-search stepper for tuning {P, I, D} automatically.
     */
    public static final class AutoSearchStepper {
        public static final String[] NAMES = {"P", "I", "D"};
        public static final double[] START = {12.0, 0.5, 0.0};
        public static final double[] MAX = {150.0, 30.0, 40.0};
        public static final double[] START_STEP = {8.0, 1.0, 2.0};
        public static final double[] MIN_STEP = {0.5, 0.1, 0.2};
        public static final int MAX_TRIALS = 40;
        public static final double IMPROVE_MARGIN = 0.97;

        private final double[] best = new double[3];
        private final double[] stepSize = new double[3];
        private final boolean[] done = new boolean[3];
        private final double[] candidate = new double[3];

        private double bestCost = Double.MAX_VALUE;
        private boolean baselineRun = true;
        private int param = 0;
        private int dir = 1;
        private int trialCount = 0;
        private boolean finished = false;

        public AutoSearchStepper() {
            reset();
        }

        public void reset() {
            System.arraycopy(START, 0, best, 0, 3);
            System.arraycopy(START_STEP, 0, stepSize, 0, 3);
            for (int n = 0; n < 3; n++) {
                done[n] = false;
            }
            System.arraycopy(best, 0, candidate, 0, 3);
            bestCost = Double.MAX_VALUE;
            baselineRun = true;
            param = 0;
            dir = 1;
            trialCount = 0;
            finished = false;
        }

        /** Prepares the next candidate into {@link #getCandidate()}. Returns false when converged/capped. */
        public boolean prepareNextCandidate() {
            if (finished || trialCount >= MAX_TRIALS) {
                finished = true;
                return false;
            }
            System.arraycopy(best, 0, candidate, 0, 3);
            if (!baselineRun) {
                boolean moved = false;
                for (int guard = 0; guard < 9; guard++) {
                    if (allDone()) {
                        finished = true;
                        return false;
                    }
                    if (done[param]) {
                        advanceParam();
                        continue;
                    }
                    double v = Math.max(0.0, Math.min(MAX[param], best[param] + dir * stepSize[param]));
                    if (Math.abs(v - best[param]) < 1e-9) {
                        rejectDirection();
                        continue;
                    }
                    candidate[param] = v;
                    moved = true;
                    break;
                }
                if (!moved) {
                    finished = true;
                    return false;
                }
            }
            trialCount++;
            return true;
        }

        /** Records the cost of the active candidate trial and returns a short status note. */
        public String recordResult(double cost) {
            if (baselineRun) {
                bestCost = cost;
                baselineRun = false;
                return "baseline";
            }
            if (cost < bestCost * IMPROVE_MARGIN) {
                bestCost = cost;
                best[param] = candidate[param];
                return "improved " + NAMES[param];
            }
            String note = "rejected " + NAMES[param];
            rejectDirection();
            return note;
        }

        private void rejectDirection() {
            if (dir == 1) {
                dir = -1;
            } else {
                stepSize[param] /= 2.0;
                if (stepSize[param] < MIN_STEP[param]) {
                    done[param] = true;
                }
                advanceParam();
            }
        }

        private void advanceParam() {
            param = (param + 1) % 3;
            dir = 1;
        }

        private boolean allDone() {
            return done[0] && done[1] && done[2];
        }

        public double[] getCandidate() {
            return candidate.clone();
        }

        public double[] getBest() {
            return best.clone();
        }

        public double getBestCost() {
            return bestCost;
        }

        public int getTrialCount() {
            return trialCount;
        }

        public int getCurrentParamIndex() {
            return param;
        }

        public double getCurrentStepSize() {
            return stepSize[param];
        }

        public boolean isFinished() {
            return finished;
        }
    }
}
