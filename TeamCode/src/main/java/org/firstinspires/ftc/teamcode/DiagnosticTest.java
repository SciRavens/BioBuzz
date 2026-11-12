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
 * Allows the driver to cycle through individual subsystem test modes or run
 * a full TeleOp test using button combos on Gamepad 1.
 *
 * ═══════════════════════════════════════════════════════════════════
 *  MODE SWITCHING  (Gamepad 1 — hold BOTH bumpers + press face button)
 * ═══════════════════════════════════════════════════════════════════
 *   LB + RB + A  →  [1] DRIVETRAIN mode
 *   LB + RB + B  →  [2] SHOOTER mode
 *   LB + RB + X  →  [3] INTAKE mode
 *   LB + RB + Y  →  [4] FULL TELEOP mode
 *
 * ═══════════════════════════════════════════════════════════════════
 *  CONTROLS INSIDE EACH MODE
 * ═══════════════════════════════════════════════════════════════════
 *
 *  [1] DRIVETRAIN
 *    Left Stick     — Strafe / Forward-Back
 *    Right Stick X  — Turn
 *    Right Trigger  — Slow-mode (25% power)
 *    A              — Reset odometry pose to (0, 0, 0)
 *
 *  [2] SHOOTER
 *    A              — Spin up (Close RPM 970)
 *    B              — Spin up (Mid RPM 1050)
 *    Y              — Spin up (Far RPM 1375)
 *    X              — STOP flywheel
 *    D-Pad Up       — RPM +25
 *    D-Pad Down     — RPM −25
 *    Right Trigger  — Reverse shooter (momentary, returns on release)
 *
 *  [3] INTAKE
 *    A              — Run intake forward (0.75)
 *    B              — Feed / shoot artifacts (0.50)
 *    Y              — Run intake reverse (−0.75)
 *    X              — STOP intake
 *
 *  [4] FULL TELEOP
 *    Full competition drive & mechanism controls. Supports both dual-driver
 *    (GP1 drive + GP2 mechanisms) and single-driver (GP1 solo) setups.
 *    - Drive: GP1 Left Stick (xy), GP1 Right Stick (turn), GP1 RT (slow-mode)
 *    - Shooter: GP2 X (Far), GP2 B (Close), GP2 A (Stop) / GP1 D-Pad (solo)
 *    - Intake: GP2 LB (Intake), GP2 LT (Reverse) / GP1 LT (solo)
 *    - Rapid Fire: GP2 Y or GP1 RB (when LB not held)
 *    - Manual Shoot: GP2 RB
 *    - Cargo Reset: GP2 or GP1 D-Pad Down
 */
@TeleOp(name = "Diagnostic Test", group = "Test")
public class DiagnosticTest extends OpMode {

    // ── Mode Enum ──────────────────────────────────────────────────
    public enum Mode {
        DRIVETRAIN("[1] DRIVETRAIN"),
        SHOOTER("[2] SHOOTER"),
        INTAKE("[3] INTAKE"),
        TELEOP("[4] FULL TELEOP");

        public final String label;
        Mode(String label) { this.label = label; }
    }

    private Mode currentMode = Mode.DRIVETRAIN;

    // ── Subsystems ─────────────────────────────────────────────────
    private Follower follower;
    private Robot robot;

    // Debounce Timers (separated to prevent control cross-interference)
    private Timer modeSwitchTimer;
    private Timer dpadDebounce;
    private Timer actionDebounce;

    // TeleOp / Rapid Fire Timers
    private Timer rapidTimer;
    private Timer shootIntervalTimer;
    private boolean isRapidFireOn = false;

    // Shooter state
    private int manualRPM = 970;
    private boolean wasReversing = false;

    private static final double DEAD_ZONE = 0.1;

    // ── Lifecycle ──────────────────────────────────────────────────
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        robot = new Robot(hardwareMap, telemetry);

        modeSwitchTimer = new Timer();
        modeSwitchTimer.reset();

        dpadDebounce = new Timer();
        dpadDebounce.reset();

        actionDebounce = new Timer();
        actionDebounce.reset();

        rapidTimer = new Timer();
        shootIntervalTimer = new Timer();

        telemetry.addLine("=== DIAGNOSTIC TEST INITIALIZED ===");
        telemetry.addLine("Hold LB+RB + face button to switch modes:");
        telemetry.addLine("  A=Drive  B=Shooter  X=Intake  Y=Full TeleOp");
        telemetry.update();
    }

    @Override
    public void start() {
        modeSwitchTimer.reset();
        dpadDebounce.reset();
        actionDebounce.reset();
        rapidTimer.reset();
        shootIntervalTimer.reset();
    }

    @Override
    public void loop() {
        // ── 1. Mode Switching Combo (LB + RB + face button) ────────
        if (gamepad1.left_bumper && gamepad1.right_bumper && modeSwitchTimer.milliseconds() > 400) {
            if (gamepad1.a) {
                switchMode(Mode.DRIVETRAIN);
            } else if (gamepad1.b) {
                switchMode(Mode.SHOOTER);
            } else if (gamepad1.x) {
                switchMode(Mode.INTAKE);
            } else if (gamepad1.y) {
                switchMode(Mode.TELEOP);
            }
        }

        // ── 2. Header Telemetry ────────────────────────────────────
        telemetry.addLine("=================================");
        telemetry.addData(">> ACTIVE MODE", currentMode.label);
        telemetry.addLine("Switch Mode: hold LB+RB + A/B/X/Y");
        telemetry.addLine("---------------------------------");

        // ── 3. Dispatch to Active Mode Handler ─────────────────────
        switch (currentMode) {
            case DRIVETRAIN: loopDrivetrain(); break;
            case SHOOTER:    loopShooter();    break;
            case INTAKE:     loopIntake();     break;
            case TELEOP:     loopTeleop();     break;
        }

        // ── 4. Unified Telemetry Push ──────────────────────────────
        telemetry.addLine("=================================");
        telemetry.update();
    }

    // ── Mode Switching Helper ──────────────────────────────────────
    public void switchMode(Mode newMode) {
        stopAllSubsystems();
        isRapidFireOn = false;
        wasReversing = false;
        currentMode = newMode;
        modeSwitchTimer.reset();

        // Haptic feedback to confirm switch
        gamepad1.rumble(200);
        gamepad2.rumble(200);
    }

    public void stopAllSubsystems() {
        if (robot != null) {
            if (robot.shooter != null) robot.shooter.stopFlyWheel();
            if (robot.intake != null) robot.intake.intakeStop();
        }
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
    }

    // ════════════════════════════════════════════════════════════════
    //  [1] DRIVETRAIN MODE
    // ════════════════════════════════════════════════════════════════
    private void loopDrivetrain() {
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double scale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.manual(yInput * scale, xInput * scale, turnInput * scale);
        follower.update();

        // A = reset pose (only when not in mode-switch combo)
        if (gamepad1.a && !gamepad1.left_bumper && actionDebounce.milliseconds() > 400) {
            follower.setPose(new Pose(0, 0, 0));
            actionDebounce.reset();
        }

        Pose p = follower.pose();
        telemetry.addLine("[DRIVETRAIN CONTROLS]");
        telemetry.addLine("  Left Stick: Drive | Right Stick: Turn");
        telemetry.addLine("  Right Trigger: Slow Mode (25%) | A: Reset Pose");
        telemetry.addData("Pose (X, Y)", "(%.2f, %.2f) in", p.x(), p.y());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(p.heading()));
        telemetry.addData("Speed Mode", scale == 0.25 ? "SLOW (25%)" : "FULL (100%)");
        telemetry.addData("Power Inputs", "Fwd: %.2f | Str: %.2f | Turn: %.2f", yInput, xInput, turnInput);
    }

    // ════════════════════════════════════════════════════════════════
    //  [2] SHOOTER MODE
    // ════════════════════════════════════════════════════════════════
    private void loopShooter() {
        // Face button presets (only when not mode switching)
        if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
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

        // D-Pad RPM tuning
        if (dpadDebounce.milliseconds() > 200) {
            if (gamepad1.dpad_up) {
                manualRPM += 25;
                robot.shooter.setRPM(manualRPM);
                dpadDebounce.reset();
            } else if (gamepad1.dpad_down) {
                manualRPM = Math.max(0, manualRPM - 25);
                robot.shooter.setRPM(manualRPM);
                dpadDebounce.reset();
            }
        }

        // Right trigger = momentary reverse
        if (gamepad1.right_trigger > 0.5) {
            robot.shooter.startReverseShoot();
            wasReversing = true;
        } else if (wasReversing) {
            // Returned from reverse: restore target RPM or stop
            if (manualRPM > 0) {
                robot.shooter.setRPM(manualRPM);
            } else {
                robot.shooter.stopFlyWheel();
            }
            wasReversing = false;
        }

        double actualVelocity = robot.shooter.shooterMotor.getVelocity();
        boolean atSpeed = manualRPM > 0 && actualVelocity >= manualRPM;

        telemetry.addLine("[SHOOTER CONTROLS]");
        telemetry.addLine("  A: Close(970) | B: Mid(1050) | Y: Far(1375) | X: STOP");
        telemetry.addLine("  D-Pad Up/Down: +/- 25 RPM | RT: Momentary Reverse");
        telemetry.addData("Target RPM", manualRPM);
        telemetry.addData("Actual Velocity", "%.1f RPM", actualVelocity);
        telemetry.addData("Status", atSpeed ? "READY TO SHOOT" : (manualRPM > 0 ? "SPINNING UP..." : "IDLE / OFF"));
        telemetry.addData("Motor Power", "%.3f", robot.shooter.shooterMotor.getPower());
    }

    // ════════════════════════════════════════════════════════════════
    //  [3] INTAKE MODE
    // ════════════════════════════════════════════════════════════════
    private void loopIntake() {
        if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
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
            intakeState = "INTAKING FORWARD (0.75)";
        } else if (intakePower > 0.1) {
            intakeState = "FEEDING / SHOOTING (0.50)";
        } else if (intakePower < -0.1) {
            intakeState = "REVERSING (-0.75)";
        } else {
            intakeState = "STOPPED (0.00)";
        }

        telemetry.addLine("[INTAKE CONTROLS]");
        telemetry.addLine("  A: Forward(0.75) | B: Feed/Shoot(0.50)");
        telemetry.addLine("  Y: Reverse(-0.75) | X: STOP");
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Motor Power", "%.2f", intakePower);
        telemetry.addData("Motor Velocity", "%.1f ticks/sec", robot.intake.intakeMotor.getVelocity());
    }

    // ════════════════════════════════════════════════════════════════
    //  [4] FULL TELEOP MODE
    // ════════════════════════════════════════════════════════════════
    private void loopTeleop() {
        // ── Alliance Toggle (GP1 Y / X when not in mode switch) ───
        if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            if (gamepad1.y && actionDebounce.milliseconds() > 400) {
                Robot.updateAlliance("BLUE");
                actionDebounce.reset();
            } else if (gamepad1.x && actionDebounce.milliseconds() > 400) {
                Robot.updateAlliance("RED");
                actionDebounce.reset();
            }
        }

        // ── Drivetrain (Pedro 3 Manual Drive) ──────────────────────
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.manual(yInput * powerScale, xInput * powerScale, turnInput * powerScale);
        follower.update();

        Pose currentPose = follower.pose();
        double distanceToGoal = robot.getDistanceFromGoal(follower);

        // ── Voltage Cargo Tracker ──────────────────────────────────
        double drivetrainMagnitude = (Math.abs(xInput) + Math.abs(yInput) + Math.abs(turnInput)) * powerScale;
        robot.cargoTracker.update(drivetrainMagnitude);

        if (robot.cargoTracker.checkAndClearFullCapacityAlert()) {
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }

        // Queue clear override (GP2 D-Pad Down or GP1 D-Pad Down)
        if (gamepad2.dpad_down || (!gamepad1.left_bumper && gamepad1.dpad_down)) {
            robot.cargoTracker.clear();
        }

        // ── Shooter Controls ───────────────────────────────────────
        if (gamepad2.x || (!gamepad1.left_bumper && gamepad1.dpad_up)) {
            robot.shooter.startFarShoot();
        } else if ((gamepad2.b && !isRapidFireOn) || (!gamepad1.left_bumper && gamepad1.dpad_right)) {
            robot.shooter.startCloseShoot();
        } else if (gamepad2.a || (!gamepad1.left_bumper && gamepad1.dpad_left)) {
            robot.shooter.stopShoot();
        } else if (gamepad1.a && !gamepad1.left_bumper && !gamepad1.right_bumper) {
            robot.shooter.startHumanIntake();
        }

        // ── Intake Controls ────────────────────────────────────────
        boolean gp1SoloIntake = !gamepad1.left_bumper && gamepad1.left_trigger > 0.5;

        if (gamepad2.left_bumper || gp1SoloIntake) {
            robot.intake.startIntakeOnly();
        } else if (gamepad2.left_trigger > 0.5) {
            robot.intake.startReverseIntake();
        } else if (!isRapidFireOn && !gamepad2.right_bumper) {
            robot.intake.stopIntake();
        }

        // ── Rapid Fire Sequence ────────────────────────────────────
        // Guarded against accidental RB trigger immediately following mode switch
        boolean gp1RapidPressed = gamepad1.right_bumper && !gamepad1.left_bumper && modeSwitchTimer.milliseconds() > 500;
        boolean triggerRapid = gamepad2.y || gp1RapidPressed;

        if (triggerRapid && !isRapidFireOn) {
            isRapidFireOn = true;
            robot.shooter.startMidShoot();
            rapidTimer.reset();
            shootIntervalTimer.reset();
            gamepad1.rumble(200);
        }

        if (isRapidFireOn) {
            if (robot.shooter.reachedSpeed()) {
                robot.intake.shootArtifacts();
                if (shootIntervalTimer.milliseconds() > 450) {
                    if (!robot.cargoTracker.isEmpty()) {
                        robot.cargoTracker.popShooterCargo();
                    }
                    shootIntervalTimer.reset();
                    // Rumble on each ball discharge
                    gamepad1.rumble(150);
                    gamepad2.rumble(150);
                }
            }
            if (rapidTimer.milliseconds() >= 5750) {
                robot.shooter.stopFlyWheel();
                robot.intake.intakeStop();
                robot.cargoTracker.clear();
                isRapidFireOn = false;
            }
        } else {
            // Manual Trigger Shooting (GP2 RB)
            if (gamepad2.right_bumper) {
                if (robot.shooter.reachedSpeed()) {
                    robot.intake.shootArtifacts();
                    if (shootIntervalTimer.milliseconds() > 450) {
                        if (!robot.cargoTracker.isEmpty()) {
                            robot.cargoTracker.popShooterCargo();
                        }
                        shootIntervalTimer.reset();
                        gamepad1.rumble(150);
                        gamepad2.rumble(150);
                    }
                }
            }
        }

        // ── Telemetry ──────────────────────────────────────────────
        telemetry.addLine("[TELEOP INTEGRATION STATUS]");
        telemetry.addData("Alliance", Robot.currentAlliance);
        telemetry.addData("Rapid Fire Active", isRapidFireOn ? "ON" : "OFF");
        telemetry.addData("Goal Distance", "%.1f in", distanceToGoal);
        telemetry.addData("Flywheel Velocity", "%.1f RPM", robot.shooter.shooterMotor.getVelocity());
        telemetry.addData("Intake Power", "%.2f", robot.intake.intakeMotor.getPower());
        telemetry.addData("Pose", "(%.1f, %.1f) @ %.1f deg", currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
        robot.cargoTracker.addTelemetry(telemetry);
    }

    // ── Getters for Inspection / Testing ───────────────────────────
    public Mode getCurrentMode() { return currentMode; }
    public boolean isRapidFireOn() { return isRapidFireOn; }
    public int getManualRPM() { return manualRPM; }

    // ── Cleanup ────────────────────────────────────────────────────
    @Override
    public void stop() {
        stopAllSubsystems();
    }
}
