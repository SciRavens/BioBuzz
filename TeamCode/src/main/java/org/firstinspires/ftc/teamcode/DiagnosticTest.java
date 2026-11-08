package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Universal Diagnostic Test OpMode.
 *
 * Allows the driver to cycle through different robot subsystem test modes
 * using button combos on Gamepad 1. Inside each mode, the same set of
 * buttons (A, B, X, Y, bumpers, triggers, sticks) are reused to exercise
 * that subsystem's specific functions.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  MODE SWITCHING  (Gamepad 1 — hold BOTH bumpers + press a face button)
 * ═══════════════════════════════════════════════════════════════════
 *   LB + RB + A  →  DRIVETRAIN mode
 *   LB + RB + B  →  SHOOTER mode
 *   LB + RB + X  →  INTAKE mode
 *   LB + RB + Y  →  CARGO TRACKER mode
 *
 * ═══════════════════════════════════════════════════════════════════
 *  CONTROLS INSIDE EACH MODE  (active when mode is selected)
 * ═══════════════════════════════════════════════════════════════════
 *
 *  [DRIVETRAIN]
 *    Left Stick     — Strafe / Forward-Back
 *    Right Stick X  — Turn
 *    Right Trigger  — Slow-mode (25% power)
 *    A              — Reset odometry pose to (0,0,0)
 *
 *  [SHOOTER]
 *    A              — Spin up (Close RPM 970)
 *    B              — Spin up (Mid RPM 1050)
 *    Y              — Spin up (Far RPM 1375)
 *    X              — STOP flywheel
 *    D-Pad Up       — RPM +25
 *    D-Pad Down     — RPM −25
 *    Right Trigger  — Reverse shooter
 *
 *  [INTAKE]
 *    A              — Run intake forward (0.75)
 *    B              — Feed / shoot artifacts (0.50)
 *    Y              — Run intake reverse (−0.75)
 *    X              — STOP intake
 *
 *  [CARGO TRACKER]
 *    A              — Force-add a POLLEN to the queue
 *    B              — Force-add a NECTAR to the queue
 *    X              — Pop from shooter end (FIFO front)
 *    Y              — Clear entire cargo queue
 *    Left Stick Y   — Simulates drivetrain magnitude for voltage update
 */
@TeleOp(name = "Diagnostic Test", group = "Test")
public class DiagnosticTest extends OpMode {

    // ── Mode Enum ──────────────────────────────────────────────────
    private enum Mode {
        DRIVETRAIN("🚗 DRIVETRAIN"),
        SHOOTER("🎯 SHOOTER"),
        INTAKE("📥 INTAKE"),
        CARGO_TRACKER("📦 CARGO TRACKER");

        final String label;
        Mode(String label) { this.label = label; }
    }

    private Mode currentMode = Mode.DRIVETRAIN;

    // ── Subsystems ─────────────────────────────────────────────────
    private Follower follower;
    private Robot robot;
    private Timer buttonDebounce;

    // Shooter manual RPM (for D-Pad tuning)
    private int manualRPM = 970;

    // ── Lifecycle ──────────────────────────────────────────────────
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        robot = new Robot(hardwareMap, telemetry);

        buttonDebounce = new Timer();
        buttonDebounce.reset();

        telemetry.addLine("═══ DIAGNOSTIC TEST INITIALIZED ═══");
        telemetry.addLine("Hold LB+RB + face button to switch modes:");
        telemetry.addLine("  A=Drive  B=Shooter  X=Intake  Y=Cargo");
        telemetry.update();
    }

    @Override
    public void start() {
        buttonDebounce.reset();
    }

    @Override
    public void loop() {
        // ── Mode Switching (LB + RB + face button) ─────────────────
        if (gamepad1.left_bumper && gamepad1.right_bumper && buttonDebounce.milliseconds() > 400) {
            if (gamepad1.a) {
                switchMode(Mode.DRIVETRAIN);
            } else if (gamepad1.b) {
                switchMode(Mode.SHOOTER);
            } else if (gamepad1.x) {
                switchMode(Mode.INTAKE);
            } else if (gamepad1.y) {
                switchMode(Mode.CARGO_TRACKER);
            }
        }

        // ── Dispatch to active mode handler ────────────────────────
        switch (currentMode) {
            case DRIVETRAIN:    loopDrivetrain();    break;
            case SHOOTER:       loopShooter();       break;
            case INTAKE:        loopIntake();        break;
            case CARGO_TRACKER: loopCargoTracker();  break;
        }

        // ── Common Telemetry Header ────────────────────────────────
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addData("▶ ACTIVE MODE", currentMode.label);
        telemetry.addLine("Switch: hold LB+RB + A/B/X/Y");
        telemetry.addLine("═══════════════════════════════════");
    }

    // ── Mode Switching Helper ──────────────────────────────────────
    private void switchMode(Mode newMode) {
        // Stop everything when switching modes for safety
        stopAllSubsystems();
        currentMode = newMode;
        buttonDebounce.reset();

        // Rumble to confirm mode switch
        gamepad1.rumble(200);
    }

    private void stopAllSubsystems() {
        robot.shooter.stopFlyWheel();
        robot.intake.intakeStop();
        follower.manual(0, 0, 0);
        follower.update();
    }

    // ════════════════════════════════════════════════════════════════
    //  DRIVETRAIN MODE
    // ════════════════════════════════════════════════════════════════
    private void loopDrivetrain() {
        double deadzone = 0.1;
        double xInput = Math.abs(gamepad1.left_stick_x) > deadzone ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > deadzone ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > deadzone ? -gamepad1.right_stick_x : 0;

        double scale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.manual(yInput * scale, xInput * scale, turnInput * scale);
        follower.update();

        // A = reset pose
        if (gamepad1.a && !gamepad1.left_bumper && buttonDebounce.milliseconds() > 400) {
            follower.setPose(new Pose(0, 0, 0));
            buttonDebounce.reset();
        }

        Pose p = follower.pose();
        telemetry.addLine("── Drivetrain Controls ──");
        telemetry.addLine("  Sticks: Drive | RT: Slow | A: Reset Pose");
        telemetry.addData("Pose", "(%.1f, %.1f) @ %.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Power Scale", scale == 0.25 ? "SLOW (25%)" : "FULL");
        telemetry.addData("Inputs (y/x/turn)", "%.2f / %.2f / %.2f", yInput, xInput, turnInput);
        telemetry.update();
    }

    // ════════════════════════════════════════════════════════════════
    //  SHOOTER MODE
    // ════════════════════════════════════════════════════════════════
    private void loopShooter() {
        // Don't process face buttons if bumpers are held (mode switching)
        if (!gamepad1.left_bumper) {
            if (gamepad1.a) {
                manualRPM = robot.shooter.shooterCloseRPM;
                robot.shooter.setRPM(manualRPM);
            } else if (gamepad1.b) {
                manualRPM = robot.shooter.autonMidRPM;
                robot.shooter.setRPM(manualRPM);
            } else if (gamepad1.y) {
                manualRPM = robot.shooter.shooterFarRPM;
                robot.shooter.setRPM(manualRPM);
            } else if (gamepad1.x) {
                robot.shooter.stopFlyWheel();
                manualRPM = 0;
            }
        }

        // D-Pad RPM tuning (debounced)
        if (buttonDebounce.milliseconds() > 200) {
            if (gamepad1.dpad_up) {
                manualRPM += 25;
                robot.shooter.setRPM(manualRPM);
                buttonDebounce.reset();
            } else if (gamepad1.dpad_down) {
                manualRPM = Math.max(0, manualRPM - 25);
                robot.shooter.setRPM(manualRPM);
                buttonDebounce.reset();
            }
        }

        // Right trigger = reverse
        if (gamepad1.right_trigger > 0.5) {
            robot.shooter.startReverseShoot();
        }

        double actualVelocity = robot.shooter.shooterMotor.getVelocity();
        boolean atSpeed = manualRPM > 0 && actualVelocity >= manualRPM;

        telemetry.addLine("── Shooter Controls ──");
        telemetry.addLine("  A: Close(970)  B: Mid(1050)  Y: Far(1375)");
        telemetry.addLine("  X: STOP  D↑/D↓: RPM±25  RT: Reverse");
        telemetry.addData("Target RPM", manualRPM);
        telemetry.addData("Actual Velocity", "%.1f", actualVelocity);
        telemetry.addData("At Speed?", atSpeed ? "✅ READY" : "⏳ SPINNING");
        telemetry.addData("Motor Power", "%.3f", robot.shooter.shooterMotor.getPower());
        telemetry.update();
    }

    // ════════════════════════════════════════════════════════════════
    //  INTAKE MODE
    // ════════════════════════════════════════════════════════════════
    private void loopIntake() {
        if (!gamepad1.left_bumper) {
            if (gamepad1.a) {
                robot.intake.startIntake();
            } else if (gamepad1.b) {
                robot.intake.shootArtifacts();
            } else if (gamepad1.y) {
                robot.intake.startReverseIntake();
            } else if (gamepad1.x) {
                robot.intake.intakeStop();
            }
        }

        double intakePower = robot.intake.intakeMotor.getPower();
        String intakeState;
        if (intakePower > 0.6) {
            intakeState = "🟢 INTAKING (FWD)";
        } else if (intakePower > 0.1) {
            intakeState = "🔵 FEEDING (SHOOT)";
        } else if (intakePower < -0.1) {
            intakeState = "🟠 REVERSE";
        } else {
            intakeState = "⚪ STOPPED";
        }

        telemetry.addLine("── Intake Controls ──");
        telemetry.addLine("  A: Intake(0.75)  B: Feed(0.50)");
        telemetry.addLine("  Y: Reverse(-0.75)  X: STOP");
        telemetry.addData("State", intakeState);
        telemetry.addData("Motor Power", "%.2f", intakePower);
        telemetry.addData("Velocity", "%.1f", robot.intake.intakeMotor.getVelocity());
        telemetry.update();
    }

    // ════════════════════════════════════════════════════════════════
    //  CARGO TRACKER MODE
    // ════════════════════════════════════════════════════════════════
    private void loopCargoTracker() {
        // Simulate voltage tracker update with left stick as fake drivetrain magnitude
        double fakeDrive = Math.abs(gamepad1.left_stick_y);
        robot.cargoTracker.update(fakeDrive);

        if (!gamepad1.left_bumper && buttonDebounce.milliseconds() > 400) {
            if (gamepad1.a) {
                robot.cargoTracker.addCargo(VoltageCargoTracker.CargoType.POLLEN);
                buttonDebounce.reset();
            } else if (gamepad1.b) {
                robot.cargoTracker.addCargo(VoltageCargoTracker.CargoType.NECTAR);
                buttonDebounce.reset();
            } else if (gamepad1.x) {
                robot.cargoTracker.popShooterCargo();
                buttonDebounce.reset();
            } else if (gamepad1.y) {
                robot.cargoTracker.clear();
                buttonDebounce.reset();
            }
        }

        telemetry.addLine("── Cargo Tracker Controls ──");
        telemetry.addLine("  A: +Pollen  B: +Nectar");
        telemetry.addLine("  X: Pop(shoot end)  Y: Clear All");
        telemetry.addLine("  Left Stick Y: Simulate drive magnitude");
        robot.cargoTracker.addTelemetry(telemetry);
        telemetry.addData("Battery Voltage", "%.2f V", robot.getBatteryVoltage());
        telemetry.update();
    }

    // ── Cleanup ────────────────────────────────────────────────────
    @Override
    public void stop() {
        stopAllSubsystems();
    }
}
