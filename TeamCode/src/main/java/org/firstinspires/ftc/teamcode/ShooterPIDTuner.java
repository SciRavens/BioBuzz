package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manual PIDF tuner for the shooter flywheel using the motor controller's built-in
 * velocity PIDF (RUN_USING_ENCODER). Change P/I/D/F live with the gamepad, spin the
 * flywheel, and fire balls to see how far the speed dips and how fast it recovers.
 *
 * Gamepad 1:
 *   D-Pad Left/Right : select coefficient (P, I, D, F)
 *   D-Pad Up/Down    : increase / decrease the selected coefficient by the step size
 *   LB / RB          : step size smaller / larger (x0.1 / x10)
 *   X / Y            : target velocity -25 / +25 (ticks/sec)
 *   A                : spin up to target
 *   B                : stop flywheel and feed
 *   RT (hold)        : feed balls into the flywheel (rapid-fire test)
 *   Back             : reset coefficients to the values read at init
 *
 * Good rapid-fire tuning: small "dip", short "recovery", little "overshoot".
 * Start with F only (I = D = P = 0) so the flywheel gets near the target by feedforward,
 * then raise P until it recovers quickly without oscillating, then add a little I/D if needed.
 */
@TeleOp(name = "Shooter PID Tuner (Manual)", group = "Test")
public class ShooterPIDTuner extends OpMode {

    // Match Shooter.java so the tuned values apply to the real subsystem.
    private static final DcMotorSimple.Direction SHOOTER_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final String[] NAMES = {"P", "I", "D", "F"};
    private static final double TOLERANCE_FRAC = 0.02; // "at speed" band = +/-2% of target

    private DcMotorEx shooterMotor;
    private Intake intake;

    private final double[] coeff = new double[4]; // P, I, D, F
    private final double[] initial = new double[4];
    private int selected = 0;
    private double step = 1.0;

    private int targetVelocity = 1200;
    private boolean spinning = false;

    // Edge detection
    private boolean lastUp, lastDown, lastLeft, lastRight, lastLb, lastRb, lastX, lastY, lastBack;

    // Dip / recovery tracking while feeding
    private boolean feeding = false;
    private boolean dipped = false;
    private double dipMin = 0;
    private final ElapsedTime dipTimer = new ElapsedTime();
    private double lastDipDepth = 0;
    private double lastRecoveryMs = -1;

    // Spin-up tracking
    private final ElapsedTime spinTimer = new ElapsedTime();
    private double spinPeak = 0;
    private double spinUpMs = -1;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(SHOOTER_DIRECTION);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake = new Intake(hardwareMap);

        PIDFCoefficients current = shooterMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        coeff[0] = current.p;
        coeff[1] = current.i;
        coeff[2] = current.d;
        coeff[3] = current.f;
        System.arraycopy(coeff, 0, initial, 0, 4);

        telemetry.addLine("Shooter PID Tuner (Manual) ready");
        telemetry.addLine("Dpad L/R: select | Dpad U/D: change | LB/RB: step size");
        telemetry.addLine("X/Y: target -/+25 | A: spin | B: stop | RT: feed | Back: reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleInput();

        double vel = shooterMotor.getVelocity();
        trackSpinUp(vel);
        trackDip(vel);

        if (spinning) {
            shooterMotor.setVelocity(targetVelocity);
        } else {
            shooterMotor.setVelocity(0);
        }

        // Only feed while the flywheel is commanded to spin.
        if (feeding && spinning) {
            intake.feedBalls();
        } else {
            intake.stopFeeding();
        }

        showTelemetry(vel);
    }

    private void handleInput() {
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;
        boolean x = gamepad1.x, y = gamepad1.y, back = gamepad1.back;

        if (left && !lastLeft) selected = (selected + NAMES.length - 1) % NAMES.length;
        if (right && !lastRight) selected = (selected + 1) % NAMES.length;

        boolean changed = false;
        if (up && !lastUp) {
            coeff[selected] += step;
            changed = true;
        }
        if (down && !lastDown) {
            coeff[selected] = Math.max(0, coeff[selected] - step);
            changed = true;
        }
        if (back && !lastBack) {
            System.arraycopy(initial, 0, coeff, 0, 4);
            changed = true;
        }
        if (changed) applyCoefficients();

        if (lb && !lastLb) step = Math.max(0.001, step / 10.0);
        if (rb && !lastRb) step = Math.min(100.0, step * 10.0);

        if (x && !lastX) targetVelocity = Math.max(0, targetVelocity - 25);
        if (y && !lastY) targetVelocity += 25;

        if (gamepad1.a && !spinning) {
            spinning = true;
            spinTimer.reset();
            spinPeak = 0;
            spinUpMs = -1;
        }
        if (gamepad1.b) {
            spinning = false;
        }

        feeding = gamepad1.right_trigger > 0.5;

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastLb = lb; lastRb = rb; lastX = x; lastY = y; lastBack = back;
    }

    private void applyCoefficients() {
        shooterMotor.setVelocityPIDFCoefficients(coeff[0], coeff[1], coeff[2], coeff[3]);
    }

    private void trackSpinUp(double vel) {
        if (!spinning) return;
        spinPeak = Math.max(spinPeak, vel);
        if (spinUpMs < 0 && Math.abs(vel - targetVelocity) <= targetVelocity * TOLERANCE_FRAC) {
            spinUpMs = spinTimer.milliseconds();
        }
    }

    private void trackDip(double vel) {
        if (!spinning) {
            dipped = false;
            return;
        }
        double tol = targetVelocity * TOLERANCE_FRAC;
        if (feeding && !dipped && vel < targetVelocity - tol) {
            dipped = true;
            dipMin = vel;
            dipTimer.reset();
        }
        if (dipped) {
            dipMin = Math.min(dipMin, vel);
            if (Math.abs(vel - targetVelocity) <= tol) {
                lastDipDepth = targetVelocity - dipMin;
                lastRecoveryMs = dipTimer.milliseconds();
                dipped = false;
            }
        }
    }

    private void showTelemetry(double vel) {
        telemetry.addLine("=== COEFFICIENTS (selected marked >) ===");
        for (int n = 0; n < NAMES.length; n++) {
            telemetry.addData((n == selected ? "> " : "  ") + NAMES[n], "%.4f", coeff[n]);
        }
        telemetry.addData("Step size", step);
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("State", spinning ? (feeding ? "SPINNING + FEEDING" : "SPINNING") : "STOPPED");
        telemetry.addData("Target (ticks/s)", targetVelocity);
        telemetry.addData("Velocity (ticks/s)", "%.1f", vel);
        telemetry.addData("Error", "%.1f", spinning ? targetVelocity - vel : 0.0);
        telemetry.addData("Motor power", "%.2f", shooterMotor.getPower());
        telemetry.addLine("=== QUALITY ===");
        telemetry.addData("Spin-up time (ms)", spinUpMs < 0 ? "--" : String.format("%.0f", spinUpMs));
        telemetry.addData("Overshoot", "%.1f", spinning ? Math.max(0, spinPeak - targetVelocity) : 0.0);
        telemetry.addData("Last ball dip", "%.1f ticks/s", lastDipDepth);
        telemetry.addData("Last recovery (ms)", lastRecoveryMs < 0 ? "--" : String.format("%.0f", lastRecoveryMs));
        telemetry.addLine("=== PASTE INTO Shooter.java ===");
        telemetry.addData("Code", "shooterMotor.setVelocityPIDFCoefficients(%.3f, %.3f, %.3f, %.3f);",
                coeff[0], coeff[1], coeff[2], coeff[3]);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setVelocity(0);
        if (intake != null) intake.stopFeeding();
    }
}
