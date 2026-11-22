package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Unified PIDF tuner and multi-ball recovery validator for the shooter flywheel.
 *
 * Matches {@link Shooter} hardware configuration ({@link Shooter#SHOOTER_DIRECTION},
 * {@link Shooter#SHOOTER_ZERO_POWER}), opens {@link Gate} automatically when feeding,
 * and tracks per-shot velocity dips, recovery times, and steady-state offsets across
 * continuous multi-pollen bursts.
 *
 * Modes (press Start while stopped to switch):
 *   1. MANUAL     : Live gamepad tuning of P/I/D/F + manual RT continuous feed.
 *   2. AUTO       : Measures free-spin max velocity (6000 RPM / 28 ticks/rev) for F,
 *                   then runs pattern search over P/I/D.
 *   3. BURST_TEST : Spins up, opens Gate, and feeds continuously (default) or pulsed
 *                   to report shot-by-shot dip, recovery ms, and post-shot offset.
 */
@TeleOp(name = "Flywheel PIDF Tuner", group = "Test")
public class FlywheelPIDFTuner extends OpMode {

    public enum TunerMode {
        MANUAL,
        AUTO,
        BURST_TEST
    }

    private enum AutoState {
        IDLE,
        MEASURE_MAX,
        TRIAL,
        DONE
    }

    private enum BurstState {
        IDLE,
        SPINUP,
        FEEDING,
        DONE
    }

    private static final String TAG = "FlywheelPIDF";
    private static final String[] COEFF_NAMES = {"P", "I", "D", "F"};
    private static final String[] PRESET_NAMES = {"Close", "Mid", "Far"};
    private static final int[] PRESET_VELOCITIES = {1170, 1175, 1600};

    // Auto-tune timeline (seconds)
    private static final double MEASURE_SECONDS = 3.0;
    private static final double SETTLE_END = 1.5;
    private static final double FEED_START = 2.4;
    private static final double FEED_END = 3.2;
    private static final double TRIAL_END = 4.2;
    private static final double LOW_FRAC = 0.85;

    // Burst-test timeline (seconds)
    private static final double SPINUP_TIMEOUT_SEC = 3.0;
    private static final double BURST_FEED_SEC = 4.0;
    private static final double BURST_END_SEC = 4.5;

    private DcMotorEx shooterMotor;
    private Intake intake;
    private Gate gate;
    private VoltageSensor voltageSensor;

    private TunerMode mode = TunerMode.MANUAL;
    private AutoState autoState = AutoState.IDLE;
    private BurstState burstState = BurstState.IDLE;

    // PIDF coefficients: {P, I, D, F}
    private final double[] coeff = new double[4];
    private final double[] initialCoeff = new double[4];
    private int selectedCoeff = 0;
    private double stepSize = 1.0;

    private int presetIndex = 0;
    private int targetVelocity = PRESET_VELOCITIES[0];

    // Manual state
    private boolean manualSpinning = false;
    private boolean manualFeeding = false;
    private final ElapsedTime manualClock = new ElapsedTime();

    // Auto-tune state
    private boolean useBallsInAuto = false;
    private double measuredMaxVel = 0.0;
    private double velAccum = 0.0;
    private int velSamples = 0;
    private final FlywheelPIDFMetrics.AutoSearchStepper searchStepper =
            new FlywheelPIDFMetrics.AutoSearchStepper();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime trialTimer = new ElapsedTime();
    private double prevTrialSec = 0.0;
    private double iae = 0.0;
    private double peakVel = 0.0;
    private double tailSum = 0.0;
    private double tailSumSq = 0.0;
    private double steadyErrSum = 0.0;
    private int tailCount = 0;
    private double lastAutoCost = 0.0;
    private String lastAutoNote = "";

    // Burst test & shot tracking
    private boolean continuousFeed = true;
    private final FlywheelPIDFMetrics.ShotTracker shotTracker =
            new FlywheelPIDFMetrics.ShotTracker(PRESET_VELOCITIES[0]);
    private final ElapsedTime burstClock = new ElapsedTime();
    private double minBatteryVoltage = 14.0;

    // Button edge tracking
    private boolean lastStart, lastBack, lastLeftStick;
    private boolean lastUp, lastDown, lastLeft, lastRight;
    private boolean lastLb, lastRb, lastA, lastB, lastX, lastY;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(Shooter.SHOOTER_DIRECTION);
        shooterMotor.setZeroPowerBehavior(Shooter.SHOOTER_ZERO_POWER);

        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        gate.close();

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensor = sensor;
            break;
        }

        PIDFCoefficients current = shooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        coeff[0] = current.p;
        coeff[1] = current.i;
        coeff[2] = current.d;
        coeff[3] = current.f;
        System.arraycopy(coeff, 0, initialCoeff, 0, 4);

        minBatteryVoltage = readVoltage();

        telemetry.addLine("Flywheel PIDF Tuner ready (6000 RPM / 28 ticks/rev)");
        telemetry.addLine("Start: cycle mode | Back: preset | B: stop/abort");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleInput();

        double vel = shooterMotor.getVelocity();
        double voltage = readVoltage();
        if (voltage > 0.0) {
            minBatteryVoltage = Math.min(minBatteryVoltage, voltage);
        }

        switch (mode) {
            case MANUAL:
                updateManual(vel);
                break;
            case AUTO:
                updateAuto(vel);
                break;
            case BURST_TEST:
                updateBurstTest(vel);
                break;
        }

        showTelemetry(vel, voltage);
    }

    private void handleInput() {
        boolean start = gamepad1.start;
        boolean back = gamepad1.back;
        boolean leftStick = gamepad1.left_stick_button;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        if (b && !lastB) {
            stopAll("Stopped by driver");
            saveEdges(start, back, leftStick, up, down, left, right, lb, rb, a, b, x, y);
            return;
        }

        if (isIdle()) {
            if (start && !lastStart) {
                TunerMode[] modes = TunerMode.values();
                mode = modes[(mode.ordinal() + 1) % modes.length];
            }
            if (back && !lastBack) {
                presetIndex = (presetIndex + 1) % PRESET_VELOCITIES.length;
                setTargetVelocity(PRESET_VELOCITIES[presetIndex]);
            }
        }

        if (mode == TunerMode.MANUAL) {
            handleManualInput(leftStick, up, down, left, right, lb, rb, a, x, y);
        } else if (mode == TunerMode.AUTO) {
            handleAutoInput(up, down, a, x, y);
        } else {
            handleBurstInput(up, down, a, y);
        }

        saveEdges(start, back, leftStick, up, down, left, right, lb, rb, a, b, x, y);
    }

    private void saveEdges(
            boolean start, boolean back, boolean leftStick,
            boolean up, boolean down, boolean left, boolean right,
            boolean lb, boolean rb, boolean a, boolean b, boolean x, boolean y) {
        lastStart = start;
        lastBack = back;
        lastLeftStick = leftStick;
        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
        lastLb = lb;
        lastRb = rb;
        lastA = a;
        lastB = b;
        lastX = x;
        lastY = y;
    }

    private void handleManualInput(
            boolean leftStick,
            boolean up, boolean down, boolean left, boolean right,
            boolean lb, boolean rb, boolean a, boolean x, boolean y) {
        if (left && !lastLeft) {
            selectedCoeff = (selectedCoeff + COEFF_NAMES.length - 1) % COEFF_NAMES.length;
        }
        if (right && !lastRight) {
            selectedCoeff = (selectedCoeff + 1) % COEFF_NAMES.length;
        }

        boolean changed = false;
        if (up && !lastUp) {
            coeff[selectedCoeff] += stepSize;
            changed = true;
        }
        if (down && !lastDown) {
            coeff[selectedCoeff] = Math.max(0.0, coeff[selectedCoeff] - stepSize);
            changed = true;
        }
        if (leftStick && !lastLeftStick) {
            System.arraycopy(initialCoeff, 0, coeff, 0, 4);
            changed = true;
        }
        if (changed) {
            applyCoefficients();
        }

        if (lb && !lastLb) {
            stepSize = Math.max(0.001, stepSize / 10.0);
        }
        if (rb && !lastRb) {
            stepSize = Math.min(100.0, stepSize * 10.0);
        }

        if (x && !lastX) {
            setTargetVelocity(Math.max(100, targetVelocity - 25));
        }
        if (y && !lastY) {
            setTargetVelocity(targetVelocity + 25);
        }

        if (a && !lastA && !manualSpinning) {
            manualSpinning = true;
            manualFeeding = false;
            manualClock.reset();
            shotTracker.reset(targetVelocity);
            minBatteryVoltage = readVoltage();
            applyCoefficients();
        }
    }

    private void handleAutoInput(boolean up, boolean down, boolean a, boolean x, boolean y) {
        if (autoState == AutoState.IDLE || autoState == AutoState.DONE) {
            if (up && !lastUp) {
                setTargetVelocity(targetVelocity + 50);
            }
            if (down && !lastDown) {
                setTargetVelocity(Math.max(100, targetVelocity - 50));
            }
            if (y && !lastY) {
                useBallsInAuto = !useBallsInAuto;
            }
            if (a && !lastA) {
                startAutoTune();
            }
            if (x && !lastX && autoState == AutoState.DONE) {
                stopFeedingAndGate();
                shooterMotor.setVelocity(0);
                mode = TunerMode.BURST_TEST;
                burstState = BurstState.IDLE;
            }
        }
    }

    private void handleBurstInput(boolean up, boolean down, boolean a, boolean y) {
        if (burstState == BurstState.IDLE || burstState == BurstState.DONE) {
            if (up && !lastUp) {
                setTargetVelocity(targetVelocity + 25);
            }
            if (down && !lastDown) {
                setTargetVelocity(Math.max(100, targetVelocity - 25));
            }
            if (y && !lastY) {
                continuousFeed = !continuousFeed;
            }
            if (a && !lastA) {
                startBurstTest();
            }
        }
    }

    // ---------------------------------------------------------------- Mode Updates

    private void updateManual(double vel) {
        if (!manualSpinning) {
            shooterMotor.setVelocity(0);
            stopFeedingAndGate();
            return;
        }

        shooterMotor.setVelocity(targetVelocity);
        boolean wantsFeed = gamepad1.right_trigger > 0.5;
        if (wantsFeed) {
            gate.open();
            intake.feedBalls();
        } else {
            if (manualFeeding) {
                shotTracker.finishBurst(manualClock.milliseconds(), vel);
            }
            stopFeedingAndGate();
        }
        manualFeeding = wantsFeed;
        shotTracker.update(manualClock.milliseconds(), vel, wantsFeed);
    }

    private void startAutoTune() {
        searchStepper.reset();
        measuredMaxVel = 0.0;
        velAccum = 0.0;
        velSamples = 0;
        lastAutoNote = "";
        lastAutoCost = 0.0;
        minBatteryVoltage = readVoltage();

        stopFeedingAndGate();
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(1.0);
        stateTimer.reset();
        autoState = AutoState.MEASURE_MAX;
    }

    private void updateAuto(double vel) {
        switch (autoState) {
            case MEASURE_MAX:
                runMeasureMax(vel);
                break;
            case TRIAL:
                runAutoTrial(vel);
                break;
            case DONE:
                shooterMotor.setVelocity(targetVelocity);
                if (gamepad1.right_trigger > 0.5) {
                    gate.open();
                    intake.feedBalls();
                } else {
                    stopFeedingAndGate();
                }
                break;
            default:
                break;
        }
    }

    private void runMeasureMax(double vel) {
        double t = stateTimer.seconds();
        if (t > MEASURE_SECONDS - 1.0) {
            velAccum += vel;
            velSamples++;
        }
        if (t < MEASURE_SECONDS) {
            return;
        }

        measuredMaxVel = velSamples > 0 ? velAccum / velSamples : 0.0;
        shooterMotor.setPower(0);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if (measuredMaxVel < 100.0) {
            stopAll("Max velocity too low - check encoder/motor");
            return;
        }

        coeff[3] = FlywheelPIDFMetrics.computeFeedforwardF(measuredMaxVel);
        double cap = FlywheelPIDFMetrics.recommendedMaxTarget(measuredMaxVel);
        if (targetVelocity > cap) {
            setTargetVelocity((int) cap);
            lastAutoNote = "Target capped at 85% max for headroom";
        }
        beginNextAutoTrial();
    }

    private void beginNextAutoTrial() {
        if (!searchStepper.prepareNextCandidate()) {
            double[] best = searchStepper.getBest();
            coeff[0] = best[0];
            coeff[1] = best[1];
            coeff[2] = best[2];
            applyCoefficients();
            autoState = AutoState.DONE;
            Log.i(TAG, String.format(
                    "AUTO DONE P=%.3f I=%.3f D=%.3f F=%.3f cost=%.3f",
                    coeff[0], coeff[1], coeff[2], coeff[3], searchStepper.getBestCost()));
            return;
        }

        double[] candidate = searchStepper.getCandidate();
        coeff[0] = candidate[0];
        coeff[1] = candidate[1];
        coeff[2] = candidate[2];
        applyCoefficients();

        iae = 0.0;
        peakVel = 0.0;
        tailSum = 0.0;
        tailSumSq = 0.0;
        steadyErrSum = 0.0;
        tailCount = 0;
        prevTrialSec = 0.0;
        trialTimer.reset();
        autoState = AutoState.TRIAL;
    }

    private void runAutoTrial(double vel) {
        double t = trialTimer.seconds();
        double dt = Math.max(0.0, t - prevTrialSec);
        prevTrialSec = t;

        if (!Double.isFinite(vel) || vel > targetVelocity * 1.5 + 250.0) {
            endAutoTrial(1e9, "runaway/NaN");
            return;
        }

        if (t < SETTLE_END) {
            shooterMotor.setVelocity(targetVelocity * LOW_FRAC);
            stopFeedingAndGate();
            return;
        }

        shooterMotor.setVelocity(targetVelocity);
        if (useBallsInAuto && t >= FEED_START && t < FEED_END) {
            gate.open();
            intake.feedBalls();
        } else {
            stopFeedingAndGate();
        }

        iae += Math.abs(targetVelocity - vel) * dt;
        peakVel = Math.max(peakVel, vel);

        if (t > TRIAL_END - 0.6) {
            tailSum += vel;
            tailSumSq += vel * vel;
            steadyErrSum += (targetVelocity - vel);
            tailCount++;
        }

        if (t >= TRIAL_END) {
            double tailStdDev = 0.0;
            if (tailCount > 1) {
                double mean = tailSum / tailCount;
                double var = Math.max(0.0, tailSumSq / tailCount - mean * mean);
                tailStdDev = Math.sqrt(var);
            }
            double finalSteadyErr = tailCount > 0 ? steadyErrSum / tailCount : (targetVelocity - vel);
            double cost = FlywheelPIDFMetrics.computeTrialCost(
                    targetVelocity,
                    iae,
                    TRIAL_END - SETTLE_END,
                    peakVel,
                    tailStdDev,
                    finalSteadyErr);
            endAutoTrial(cost, "");
        }
    }

    private void endAutoTrial(double cost, String extraNote) {
        stopFeedingAndGate();
        lastAutoCost = cost;
        String stepNote = searchStepper.recordResult(cost);
        lastAutoNote = extraNote.isEmpty() ? stepNote : stepNote + " (" + extraNote + ")";
        Log.i(TAG, String.format(
                "trial %d P=%.2f I=%.2f D=%.2f F=%.3f cost=%.3f %s",
                searchStepper.getTrialCount(), coeff[0], coeff[1], coeff[2], coeff[3], cost, lastAutoNote));
        beginNextAutoTrial();
    }

    private void startBurstTest() {
        applyCoefficients();
        shotTracker.reset(targetVelocity);
        minBatteryVoltage = readVoltage();
        stopFeedingAndGate();
        stateTimer.reset();
        burstClock.reset();
        burstState = BurstState.SPINUP;
    }

    private void updateBurstTest(double vel) {
        switch (burstState) {
            case SPINUP:
                shooterMotor.setVelocity(targetVelocity);
                stopFeedingAndGate();
                shotTracker.update(burstClock.milliseconds(), vel, false);
                if (shotTracker.isArmedOrActive() || stateTimer.seconds() >= SPINUP_TIMEOUT_SEC) {
                    stateTimer.reset();
                    burstState = BurstState.FEEDING;
                }
                break;

            case FEEDING:
                shooterMotor.setVelocity(targetVelocity);
                double t = stateTimer.seconds();
                boolean feedNow = continuousFeed
                        ? (t < BURST_FEED_SEC)
                        : (t < BURST_FEED_SEC && (t % 1.0) < 0.45);
                if (feedNow) {
                    gate.open();
                    intake.feedBalls();
                } else {
                    stopFeedingAndGate();
                }
                shotTracker.update(burstClock.milliseconds(), vel, feedNow);

                if (t >= BURST_END_SEC) {
                    stopFeedingAndGate();
                    shotTracker.finishBurst(burstClock.milliseconds(), vel);
                    shooterMotor.setVelocity(0);
                    burstState = BurstState.DONE;
                    logBurstSummary();
                }
                break;

            default:
                break;
        }
    }

    private void logBurstSummary() {
        Log.i(TAG, String.format(
                "BURST DONE target=%d shots=%d maxDip=%.1f worstRecov=%.0fms maxOffset=%.1f minV=%.2fV",
                targetVelocity,
                shotTracker.getShotCount(),
                shotTracker.getMaxDipDepth(),
                shotTracker.getWorstRecoveryMs(),
                shotTracker.getMaxAbsSteadyStateOffset(),
                minBatteryVoltage));
        for (FlywheelPIDFMetrics.ShotRecord s : shotTracker.getShots()) {
            Log.i(TAG, String.format(
                    "  Shot #%d: dip=%.1f recov=%s offset=%+.1f overshoot=%.1f",
                    s.shotNumber,
                    s.dipDepth,
                    s.recovered() ? String.format("%.0fms", s.recoveryTimeMs) : "UNRECOVERED",
                    s.steadyStateOffset,
                    s.peakOvershoot));
        }
    }

    // ---------------------------------------------------------------- Helpers

    private boolean isIdle() {
        if (mode == TunerMode.MANUAL) {
            return !manualSpinning;
        }
        if (mode == TunerMode.AUTO) {
            return autoState == AutoState.IDLE || autoState == AutoState.DONE;
        }
        return burstState == BurstState.IDLE || burstState == BurstState.DONE;
    }

    private void setTargetVelocity(int newTarget) {
        targetVelocity = Math.max(100, newTarget);
        shotTracker.reset(targetVelocity);
    }

    private void applyCoefficients() {
        shooterMotor.setVelocityPIDFCoefficients(coeff[0], coeff[1], coeff[2], coeff[3]);
    }

    private void stopFeedingAndGate() {
        intake.stopFeeding();
        gate.close();
    }

    private void stopAll(String note) {
        manualSpinning = false;
        manualFeeding = false;
        autoState = AutoState.IDLE;
        burstState = BurstState.IDLE;
        if (!note.isEmpty()) {
            lastAutoNote = note;
        }
        shooterMotor.setPower(0);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocity(0);
        stopFeedingAndGate();
    }

    private double readVoltage() {
        return voltageSensor != null ? voltageSensor.getVoltage() : 12.0;
    }

    // ---------------------------------------------------------------- Telemetry

    private void showTelemetry(double vel, double voltage) {
        telemetry.addData("Mode (Start to switch)", mode);
        telemetry.addData("Target (ticks/s | RPM)", "%d | %.0f (Preset: %s)",
                targetVelocity,
                FlywheelPIDFMetrics.ticksPerSecToRpm(targetVelocity),
                PRESET_NAMES[presetIndex]);
        telemetry.addData("Actual (ticks/s | RPM)", "%.1f | %.0f",
                vel, FlywheelPIDFMetrics.ticksPerSecToRpm(vel));
        telemetry.addData("Error (ticks/s)", "%.1f", targetVelocity - vel);
        telemetry.addData("Power | Gate | Batt", "%.2f | %s | %.2fV (min %.2fV)",
                shooterMotor.getPower(),
                gate.isOpen() ? "OPEN" : "CLOSED",
                voltage,
                minBatteryVoltage);

        telemetry.addLine("=== PIDF COEFFICIENTS ===");
        for (int n = 0; n < COEFF_NAMES.length; n++) {
            String prefix = (mode == TunerMode.MANUAL && n == selectedCoeff) ? "> " : "  ";
            telemetry.addData(prefix + COEFF_NAMES[n], "%.4f", coeff[n]);
        }

        if (mode == TunerMode.MANUAL) {
            telemetry.addData("Step Size (LB/RB)", stepSize);
            telemetry.addData("State", manualSpinning ? (manualFeeding ? "SPIN + FEED" : "SPINNING") : "STOPPED");
            telemetry.addLine("Dpad L/R: sel | U/D: +/- | X/Y: target -/+25 | A: spin | RT: feed | LStick: reset");
        } else if (mode == TunerMode.AUTO) {
            telemetry.addData("Auto State", autoState);
            telemetry.addData("Use Balls (Y)", useBallsInAuto ? "YES (continuous pulse)" : "no (step only)");
            if (measuredMaxVel > 0.0) {
                telemetry.addData("Measured Max", "%.0f ticks/s (%.0f RPM)",
                        measuredMaxVel, FlywheelPIDFMetrics.ticksPerSecToRpm(measuredMaxVel));
            }
            if (autoState == AutoState.TRIAL || autoState == AutoState.DONE) {
                telemetry.addData("Trial", "%d / %d (tuning %s step %.2f)",
                        searchStepper.getTrialCount(),
                        FlywheelPIDFMetrics.AutoSearchStepper.MAX_TRIALS,
                        FlywheelPIDFMetrics.AutoSearchStepper.NAMES[searchStepper.getCurrentParamIndex()],
                        searchStepper.getCurrentStepSize());
                String bestCostText = searchStepper.getBestCost() == Double.MAX_VALUE
                        ? "--"
                        : String.format("%.3f", searchStepper.getBestCost());
                telemetry.addData("Best Cost | Last Cost", "%s | %.3f", bestCostText, lastAutoCost);
            }
            if (!lastAutoNote.isEmpty()) {
                telemetry.addData("Note", lastAutoNote);
            }
            telemetry.addLine(autoState == AutoState.DONE
                    ? "X: Burst Test | RT: manual feed | A: re-tune | B: stop"
                    : "A: start auto-tune | Y: toggle balls | Dpad U/D: target +/-50 | B: abort");
        } else {
            telemetry.addData("Burst State", burstState);
            telemetry.addData("Feed Mode (Y)", continuousFeed ? "CONTINUOUS (default)" : "PULSED");
            telemetry.addLine("A: run 4s burst | Dpad U/D: target +/-25 | B: abort");
        }

        List<FlywheelPIDFMetrics.ShotRecord> shots = shotTracker.getShots();
        if (!shots.isEmpty()) {
            telemetry.addLine("=== MULTI-BALL SHOT RECOVERY ===");
            telemetry.addData("Summary", "Shots=%d MaxDip=%.1f WorstRecov=%.0fms MaxOffset=%.1f Unrec=%d",
                    shotTracker.getShotCount(),
                    shotTracker.getMaxDipDepth(),
                    shotTracker.getWorstRecoveryMs(),
                    shotTracker.getMaxAbsSteadyStateOffset(),
                    shotTracker.getUnrecoveredCount());
            for (FlywheelPIDFMetrics.ShotRecord s : shots) {
                telemetry.addData("Shot #" + s.shotNumber,
                        "Dip -%.0f | Recov %s | Offset %+.0f | Over +%.0f",
                        s.dipDepth,
                        s.recovered() ? String.format("%.0fms", s.recoveryTimeMs) : "NONE",
                        s.steadyStateOffset,
                        s.peakOvershoot);
            }
        }

        telemetry.addLine("=== PASTE INTO Shooter.java ===");
        telemetry.addData("P/I/D/F", "TUNED_P=%.3f; TUNED_I=%.3f; TUNED_D=%.3f; TUNED_F=%.3f;",
                coeff[0], coeff[1], coeff[2], coeff[3]);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooterMotor.setVelocity(0);
        }
        if (intake != null) {
            intake.stopFeeding();
        }
        if (gate != null) {
            gate.close();
        }
    }
}
