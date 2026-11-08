package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Automatic PIDF tuner for the shooter flywheel (built-in velocity PIDF, RUN_USING_ENCODER).
 *
 * How it works:
 *  1. F is measured: the flywheel runs at full power and F = 32767 / maxVelocity (ticks/s).
 *  2. P, I and D are found with a pattern search. Each trial applies a candidate, lets the
 *     flywheel settle at a low setpoint, then steps up to the target and holds. That step
 *     imitates the speed loss after a ball is fired, so a good score means fast recovery.
 *     Optionally (Y) the intake also pushes a real ball through mid-trial for a true dip.
 *  3. Each trial is scored (lower is better) on tracking error, overshoot and jitter.
 *     A candidate is only kept if it beats the best score by a margin, so noise is not chased.
 *  4. When the search converges (or hits the trial cap) the best values are applied and
 *     printed ready to paste into Shooter.java. They are also written to logcat (tag ShooterAutoPID).
 *
 * Gamepad 1:
 *   A           : start auto-tune (flywheel must be free to spin, battery charged)
 *   B           : abort / stop everything
 *   Y           : toggle "push real balls through during trials" (load balls first)
 *   Dpad Up/Down: target velocity +/-50 ticks/s (while idle). Use your real shooting speed.
 *   X           : after tuning, run a rapid-fire validation using the best values
 *   RT (hold)   : after tuning, manually feed balls to confirm
 *
 * Keep the target below ~85% of max velocity so the controller has headroom.
 */
@TeleOp(name = "Shooter PID Tuner (Auto)", group = "Test")
public class ShooterAutoPIDTuner extends OpMode {

    private static final String TAG = "ShooterAutoPID";
    private static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Search limits and step sizes, index: 0 = P, 1 = I, 2 = D
    private static final String[] NAMES = {"P", "I", "D"};
    private static final double[] MAX = {150.0, 30.0, 40.0};
    private static final double[] START = {10.0, 0.0, 0.0};
    private static final double[] START_STEP = {8.0, 1.0, 2.0};
    private static final double[] MIN_STEP = {0.5, 0.1, 0.2};
    private static final int MAX_TRIALS = 40;
    private static final double IMPROVE_MARGIN = 0.97; // must beat best by 3%

    // Trial timeline (seconds from trial start)
    private static final double SETTLE_END = 1.5; // low setpoint, not scored
    private static final double FEED_START = 2.6; // optional real-ball pulse
    private static final double FEED_END = 3.0;
    private static final double TRIAL_END = 4.2;
    private static final double LOW_FRAC = 0.85;
    private static final double MEASURE_SECONDS = 3.0;
    private static final double F_MAX_VELOCITY = 32767.0;

    private enum State { IDLE, MEASURE_MAX, TRIAL, DONE, VALIDATE }

    private DcMotorEx shooterMotor;
    private Intake intake;
    private State state = State.IDLE;

    private int targetVelocity = 1200;
    private boolean useBalls = false;
    private boolean lastA, lastY, lastX, lastUp, lastDown;

    // Measurement
    private final ElapsedTime stateTimer = new ElapsedTime();
    private double maxVelocity = 0;
    private double velAccum = 0;
    private int velSamples = 0;
    private double f = 0;

    // Search
    private final double[] best = new double[3];
    private final double[] stepSize = new double[3];
    private final boolean[] done = new boolean[3];
    private final double[] candidate = new double[3];
    private double bestCost = Double.MAX_VALUE;
    private boolean baselineRun = true;
    private int param = 0;
    private int dir = 1; // +1 try increase, -1 try decrease
    private int trialCount = 0;
    private double lastCost = 0;
    private String lastTrialNote = "";

    // Trial scoring
    private final ElapsedTime trialTimer = new ElapsedTime();
    private double prevTime;
    private double iae, peak, tailSum, tailSumSq;
    private int tailCount;
    private boolean runaway;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(SHOOTER_DIRECTION);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        handleInput();

        switch (state) {
            case MEASURE_MAX: runMeasureMax(); break;
            case TRIAL:       runTrial(); break;
            case VALIDATE:    runValidate(); break;
            case DONE:
                shooterMotor.setVelocity(targetVelocity);
                if (gamepad1.right_trigger > 0.5) intake.feedBalls(); else intake.stopFeeding();
                break;
            default:
                break;
        }
        showTelemetry();
    }

    // ---------------------------------------------------------------- input

    private void handleInput() {
        if (gamepad1.b) {
            abort("Aborted by driver");
            return;
        }
        boolean a = gamepad1.a, y = gamepad1.y, x = gamepad1.x;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;

        if (a && !lastA && (state == State.IDLE || state == State.DONE)) startTuning();
        if (y && !lastY && (state == State.IDLE || state == State.DONE)) useBalls = !useBalls;
        if (x && !lastX && state == State.DONE) {
            state = State.VALIDATE;
            stateTimer.reset();
        }
        if (state == State.IDLE) {
            if (up && !lastUp) targetVelocity += 50;
            if (down && !lastDown) targetVelocity = Math.max(100, targetVelocity - 50);
        }
        lastA = a; lastY = y; lastX = x; lastUp = up; lastDown = down;
    }

    private void abort(String note) {
        shooterMotor.setPower(0);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocity(0);
        intake.stopFeeding();
        lastTrialNote = note;
        state = State.IDLE;
    }

    // ---------------------------------------------------------------- phase 1: F

    private void startTuning() {
        System.arraycopy(START, 0, best, 0, 3);
        System.arraycopy(START_STEP, 0, stepSize, 0, 3);
        for (int n = 0; n < 3; n++) done[n] = false;
        bestCost = Double.MAX_VALUE;
        baselineRun = true;
        param = 0;
        dir = 1;
        trialCount = 0;
        maxVelocity = 0;
        velAccum = 0;
        velSamples = 0;
        lastTrialNote = "";
        // Full power with no velocity control to find the true maximum speed.
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(1.0);
        stateTimer.reset();
        state = State.MEASURE_MAX;
    }

    private void runMeasureMax() {
        double t = stateTimer.seconds();
        // Average the last second, once the wheel is up to speed.
        if (t > MEASURE_SECONDS - 1.0) {
            velAccum += shooterMotor.getVelocity();
            velSamples++;
        }
        if (t < MEASURE_SECONDS) return;

        maxVelocity = velSamples > 0 ? velAccum / velSamples : 0;
        shooterMotor.setPower(0);
        if (maxVelocity < 100) {
            abort("Max velocity too low - check encoder/motor");
            return;
        }
        f = F_MAX_VELOCITY / maxVelocity;
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (targetVelocity > maxVelocity * 0.85) {
            targetVelocity = (int) (maxVelocity * 0.85);
            lastTrialNote = "Target lowered to 85% of max velocity";
        }
        beginNextTrial();
    }

    // ---------------------------------------------------------------- phase 2: search

    /** Builds the next candidate to test, or finishes if the search is converged. */
    private void beginNextTrial() {
        if (trialCount >= MAX_TRIALS) {
            finish();
            return;
        }
        System.arraycopy(best, 0, candidate, 0, 3);

        if (!baselineRun) {
            // Find a (param, dir) that produces a different, in-range candidate.
            for (int guard = 0; guard < 8; guard++) {
                if (allDone()) {
                    finish();
                    return;
                }
                if (done[param]) {
                    advanceParam();
                    continue;
                }
                double v = best[param] + dir * stepSize[param];
                v = Math.max(0, Math.min(MAX[param], v));
                if (v == best[param]) {
                    rejectDirection();
                    continue;
                }
                candidate[param] = v;
                break;
            }
            if (candidate[param] == best[param]) { // couldn't find any move
                finish();
                return;
            }
        }
        startTrial();
    }

    private boolean allDone() {
        return done[0] && done[1] && done[2];
    }

    /** Current direction did not help: try the other one, or shrink the step and move on. */
    private void rejectDirection() {
        if (dir == 1) {
            dir = -1;
        } else {
            stepSize[param] /= 2.0;
            if (stepSize[param] < MIN_STEP[param]) done[param] = true;
            advanceParam();
        }
    }

    private void advanceParam() {
        param = (param + 1) % 3;
        dir = 1;
    }

    private void startTrial() {
        trialCount++;
        shooterMotor.setVelocityPIDFCoefficients(candidate[0], candidate[1], candidate[2], f);
        iae = 0;
        peak = 0;
        tailSum = 0;
        tailSumSq = 0;
        tailCount = 0;
        runaway = false;
        trialTimer.reset();
        prevTime = 0;
        state = State.TRIAL;
    }

    private void runTrial() {
        double t = trialTimer.seconds();
        double dt = t - prevTime;
        prevTime = t;
        double vel = shooterMotor.getVelocity();

        if (Double.isNaN(vel) || vel > targetVelocity * 1.5 + 200) {
            runaway = true;
        }
        if (runaway) {
            endTrial(1e9, "runaway/NaN");
            return;
        }

        if (t < SETTLE_END) {
            shooterMotor.setVelocity(targetVelocity * LOW_FRAC);
            intake.stopFeeding();
            return;
        }

        shooterMotor.setVelocity(targetVelocity);
        if (useBalls && t >= FEED_START && t < FEED_END) {
            intake.feedBalls();
        } else {
            intake.stopFeeding();
        }

        iae += Math.abs(targetVelocity - vel) * dt;
        peak = Math.max(peak, vel);
        if (t > TRIAL_END - 0.6) {
            tailSum += vel;
            tailSumSq += vel * vel;
            tailCount++;
        }

        if (t >= TRIAL_END) {
            endTrial(scoreTrial(), "");
        }
    }

    /** Lower is better, roughly "percent of target error". */
    private double scoreTrial() {
        double window = TRIAL_END - SETTLE_END;
        double tracking = 100.0 * iae / (targetVelocity * window);
        double overshoot = 100.0 * Math.max(0, peak - targetVelocity) / targetVelocity;
        double jitter = 0;
        if (tailCount > 1) {
            double mean = tailSum / tailCount;
            double var = Math.max(0, tailSumSq / tailCount - mean * mean);
            jitter = 100.0 * Math.sqrt(var) / targetVelocity;
        }
        return tracking + 1.5 * overshoot + 0.5 * jitter;
    }

    private void endTrial(double cost, String note) {
        intake.stopFeeding();
        lastCost = cost;
        Log.i(TAG, String.format("trial %d P=%.2f I=%.2f D=%.2f F=%.3f cost=%.3f %s",
                trialCount, candidate[0], candidate[1], candidate[2], f, cost, note));

        if (baselineRun) {
            bestCost = cost;
            baselineRun = false;
            lastTrialNote = "baseline";
        } else if (cost < bestCost * IMPROVE_MARGIN) {
            bestCost = cost;
            best[param] = candidate[param]; // keep going the same direction
            lastTrialNote = "improved " + NAMES[param];
        } else {
            lastTrialNote = "rejected " + NAMES[param] + (note.isEmpty() ? "" : " (" + note + ")");
            rejectDirection();
        }
        beginNextTrial();
    }

    // ---------------------------------------------------------------- phase 3: result

    private void finish() {
        shooterMotor.setVelocityPIDFCoefficients(best[0], best[1], best[2], f);
        stateTimer.reset();
        state = State.DONE;
        Log.i(TAG, String.format("DONE shooterMotor.setVelocityPIDFCoefficients(%.3f, %.3f, %.3f, %.3f); cost=%.3f",
                best[0], best[1], best[2], f, bestCost));
    }

    /**
     * Rapid-fire check with the best values: the intake repeatedly pushes balls through,
     * so load several balls first. Watch the dip and recovery in telemetry/logcat.
     */
    private void runValidate() {
        shooterMotor.setVelocity(targetVelocity);
        double t = stateTimer.seconds();
        // 0.4 s feed / 0.6 s pause, five times.
        boolean feedOn = t > 1.0 && t < 6.0 && ((t - 1.0) % 1.0) < 0.4;
        if (feedOn) intake.feedBalls(); else intake.stopFeeding();
        if (t >= 7.0) {
            intake.stopFeeding();
            state = State.DONE;
        }
    }

    // ---------------------------------------------------------------- telemetry

    private void showTelemetry() {
        telemetry.addData("State", state);
        telemetry.addData("Target (ticks/s)", targetVelocity);
        telemetry.addData("Velocity (ticks/s)", "%.1f", shooterMotor.getVelocity());
        telemetry.addData("Use real balls in trials", useBalls ? "YES" : "no (setpoint step only)");
        if (maxVelocity > 0) {
            telemetry.addData("Max velocity", "%.0f", maxVelocity);
            telemetry.addData("F", "%.3f", f);
        }
        if (state == State.TRIAL || state == State.DONE || state == State.VALIDATE) {
            telemetry.addData("Trial", "%d / %d", trialCount, MAX_TRIALS);
            telemetry.addData("Tuning", "%s (step %.2f)", NAMES[param], stepSize[param]);
            telemetry.addData("Best P/I/D", "%.2f / %.2f / %.2f", best[0], best[1], best[2]);
            telemetry.addData("Best cost", bestCost == Double.MAX_VALUE ? "--" : String.format("%.3f", bestCost));
            telemetry.addData("Last cost", "%.3f", lastCost);
        }
        if (!lastTrialNote.isEmpty()) telemetry.addData("Note", lastTrialNote);

        if (state == State.DONE || state == State.VALIDATE) {
            telemetry.addLine("=== RESULT (paste into Shooter.java) ===");
            telemetry.addData("Code", "shooterMotor.setVelocityPIDFCoefficients(%.3f, %.3f, %.3f, %.3f);",
                    best[0], best[1], best[2], f);
            telemetry.addLine("X: rapid-fire validation | RT: manual feed | A: re-tune | B: stop");
        } else if (state == State.IDLE) {
            telemetry.addLine("A: start auto-tune | Y: toggle real balls | Dpad U/D: target +/-50 | B: stop");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
            shooterMotor.setVelocity(0);
        }
        if (intake != null) intake.stopFeeding();
    }
}
