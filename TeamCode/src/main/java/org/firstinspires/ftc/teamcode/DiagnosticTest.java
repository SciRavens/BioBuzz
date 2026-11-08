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
 *  MODE SWITCHING  (Gamepad 1 — hold BOTH bumpers + press a face button)
 * ═══════════════════════════════════════════════════════════════════
 *   LB + RB + A  →  DRIVETRAIN mode
 *   LB + RB + B  →  SHOOTER mode
 *   LB + RB + X  →  INTAKE mode
 *   LB + RB + Y  →  FULL TELEOP mode (full competition drive + mechanisms)
 *
 * ═══════════════════════════════════════════════════════════════════
 *  CONTROLS INSIDE EACH MODE
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
 *  [FULL TELEOP]
 *    Full competition drive & mechanism controls supporting both dual gamepads
 *    (Gamepad 1 drive + Gamepad 2 mechanisms) and single-gamepad testing.
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
    private enum Mode {
        DRIVETRAIN("🚗 DRIVETRAIN"),
        SHOOTER("🎯 SHOOTER"),
        INTAKE("📥 INTAKE"),
        TELEOP("🎮 FULL TELEOP");

        final String label;
        Mode(String label) { this.label = label; }
    }

    private Mode currentMode = Mode.DRIVETRAIN;

    // ── Subsystems ─────────────────────────────────────────────────
    private Follower follower;
    private Robot robot;
    private Timer buttonDebounce;

    // TeleOp / Rapid Fire Timers
    private Timer rapidTimer;
    private Timer shootIntervalTimer;
    private boolean isRapidFireOn = false;

    // Shooter manual RPM (for D-Pad tuning in Shooter Mode)
    private int manualRPM = 970;

    private static final double DEAD_ZONE = 0.1;

    // ── Lifecycle ──────────────────────────────────────────────────
    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        robot = new Robot(hardwareMap, telemetry);

        buttonDebounce = new Timer();
        buttonDebounce.reset();

        rapidTimer = new Timer();
        shootIntervalTimer = new Timer();

        telemetry.addLine("═══ DIAGNOSTIC TEST INITIALIZED ═══");
        telemetry.addLine("Hold LB+RB + face button to switch modes:");
        telemetry.addLine("  A=Drive  B=Shooter  X=Intake  Y=Full TeleOp");
        telemetry.update();
    }

    @Override
    public void start() {
        buttonDebounce.reset();
        rapidTimer.reset();
        shootIntervalTimer.reset();
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
                switchMode(Mode.TELEOP);
            }
        }

        // ── Dispatch to active mode handler ────────────────────────
        switch (currentMode) {
            case DRIVETRAIN: loopDrivetrain(); break;
            case SHOOTER:    loopShooter();    break;
            case INTAKE:     loopIntake();     break;
            case TELEOP:     loopTeleop();     break;
        }

        // ── Common Telemetry Header ────────────────────────────────
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addData("▶ ACTIVE MODE", currentMode.label);
        telemetry.addLine("Switch Mode: hold LB+RB + A/B/X/Y");
        telemetry.addLine("═══════════════════════════════════");
    }

    // ── Mode Switching Helper ──────────────────────────────────────
    private void switchMode(Mode newMode) {
        stopAllSubsystems();
        isRapidFireOn = false;
        currentMode = newMode;
        buttonDebounce.reset();

        // Haptic feedback to confirm switch
        gamepad1.rumble(200);
        gamepad2.rumble(200);
    }

    private void stopAllSubsystems() {
        robot.shooter.stopFlyWheel();
        robot.intake.intakeStop();
        follower.manual(0, 0, 0);
        follower.update();
    }

    // ════════════════════════════════════════════════════════════════
    //  1. DRIVETRAIN MODE
    // ════════════════════════════════════════════════════════════════
    private void loopDrivetrain() {
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double scale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.manual(yInput * scale, xInput * scale, turnInput * scale);
        follower.update();

        // A = reset pose (when not in mode-switch combo)
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
    //  2. SHOOTER MODE
    // ════════════════════════════════════════════════════════════════
    private void loopShooter() {
        // Face button presets (when not mode switching)
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

        // D-Pad RPM tuning
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
    //  3. INTAKE MODE
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
    //  4. FULL TELEOP MODE
    // ════════════════════════════════════════════════════════════════
    private void loopTeleop() {
        // ── Alliance Toggle (GP1 Y / X without holding LB) ────────
        if (!gamepad1.left_bumper) {
            if (gamepad1.y) {
                Robot.updateAlliance("BLUE");
            } else if (gamepad1.x) {
                Robot.updateAlliance("RED");
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

        if (gamepad2.dpad_down || (!gamepad1.left_bumper && gamepad1.dpad_down)) {
            robot.cargoTracker.clear();
        }

        // ── Shooter Controls (GP2 primary, GP1 fallback for solo) ──
        if (gamepad2.x || (!gamepad1.left_bumper && gamepad1.dpad_up)) {
            robot.shooter.startFarShoot();
        } else if ((gamepad2.b && !isRapidFireOn) || (!gamepad1.left_bumper && gamepad1.dpad_right)) {
            robot.shooter.startCloseShoot();
        } else if (gamepad2.a || (!gamepad1.left_bumper && gamepad1.dpad_left)) {
            robot.shooter.stopShoot();
        } else if (gamepad1.a && !gamepad1.left_bumper) {
            robot.shooter.startHumanIntake();
        }

        // ── Intake Controls ────────────────────────────────────────
        boolean gp1SoloIntake = !gamepad1.left_bumper && gamepad1.left_trigger > 0.5;
        boolean gp1SoloReverse = gamepad1.left_bumper && !gamepad1.right_bumper;

        if (gamepad2.left_bumper || gp1SoloIntake) {
            robot.intake.startIntakeOnly();
        } else if (gamepad2.left_trigger > 0.5 || gp1SoloReverse) {
            robot.intake.startReverseIntake();
        } else if (!isRapidFireOn && !gamepad2.right_bumper) {
            robot.intake.stopIntake();
        }

        // ── Rapid Fire (GP2 Y or GP1 RB when LB not held) ─────────
        boolean triggerRapid = gamepad2.y || (gamepad1.right_bumper && !gamepad1.left_bumper);
        if (triggerRapid && !isRapidFireOn) {
            isRapidFireOn = true;
            robot.shooter.startMidShoot();
            rapidTimer.reset();
        }

        if (isRapidFireOn) {
            if (robot.shooter.reachedSpeed()) {
                robot.intake.shootArtifacts();
                if (shootIntervalTimer.milliseconds() > 450 && !robot.cargoTracker.isEmpty()) {
                    robot.cargoTracker.popShooterCargo();
                    shootIntervalTimer.reset();
                }
                gamepad1.rumble(500);
                gamepad2.rumble(500);
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
                    if (shootIntervalTimer.milliseconds() > 450 && !robot.cargoTracker.isEmpty()) {
                        robot.cargoTracker.popShooterCargo();
                        shootIntervalTimer.reset();
                    }
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                }
            }
        }

        // ── Telemetry ──────────────────────────────────────────────
        robot.shooter.shooterLightUpdate();
        robot.cargoTracker.addTelemetry(telemetry);
        telemetry.addData("Alliance", Robot.currentAlliance);
        telemetry.addData("Rapid Fire Active", isRapidFireOn);
        telemetry.addData("Goal Distance", "%.1f in", distanceToGoal);
        telemetry.addData("Flywheel Velocity", "%.1f RPM", robot.shooter.shooterMotor.getVelocity());
        telemetry.addData("Intake Power", "%.2f", robot.intake.intakeMotor.getPower());
        telemetry.addData("Pose", "(%.1f, %.1f) @ %.1f°", currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
        telemetry.update();
    }

    // ── Cleanup ────────────────────────────────────────────────────
    @Override
    public void stop() {
        stopAllSubsystems();
    }
}
