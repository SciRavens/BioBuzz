package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Dedicated test OpMode for Shooter and Intake mechanisms.
 * Allows driver to intake balls, spin up the shooter flywheel to target RPM,
 * and shoot balls by feeding them into the flywheel.
 * Completely free of Turret, Vision, Gate, and LED dependencies.
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestOpMode extends OpMode {

    // Presets from Shooter constants
    public static final int RPM_CLOSE = 1650;
    public static final int RPM_MID = 1700;
    public static final int RPM_FAR = 1680;

    public DcMotorEx shooterMotor;
    public Intake intake;

    private Timer buttonTimer;
    private int currentRPM = RPM_CLOSE;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intake = new Intake(hardwareMap);

        buttonTimer = new Timer();
        buttonTimer.reset();

        telemetry.addLine("=== SHOOTER TEST OPMODE INITIALIZED ===");
        telemetry.addData("Default Target RPM", currentRPM);
        telemetry.addLine("Controls (Gamepad 1):");
        telemetry.addLine("  LB (hold)  : Intake ON");
        telemetry.addLine("  LT (hold)  : Reverse Intake (Clear Jam)");
        telemetry.addLine("  RB (hold)  : Spin Up Shooter Flywheel");
        telemetry.addLine("  RT (hold)  : SHOOT (Feed ball into flywheel)");
        telemetry.addLine("  Y / A      : Target RPM +25 / -25");
        telemetry.addLine("  D-Pad ↑/→/↓: Presets 970 / 1050 / 1375 RPM");
        telemetry.addLine("  X          : Emergency Stop All Motors");
        telemetry.update();
    }

    @Override
    public void start() {
        buttonTimer.reset();
    }

    @Override
    public void loop() {
        // 1. RPM Adjustment (+25 / -25) with 300ms debounce
        if (gamepad1.y && buttonTimer.milliseconds() > 300) {
            currentRPM += 25;
            buttonTimer.reset();
        } else if (gamepad1.a && buttonTimer.milliseconds() > 300) {
            currentRPM = Math.max(0, currentRPM - 25);
            buttonTimer.reset();
        }

        // 2. RPM Presets via D-Pad
        if (gamepad1.dpad_up) {
            currentRPM = RPM_CLOSE;
        } else if (gamepad1.dpad_right) {
            currentRPM = RPM_MID;
        } else if (gamepad1.dpad_down) {
            currentRPM = RPM_FAR;
        }

        // 3. Emergency Stop (X button)
        if (gamepad1.x) {
            shooterMotor.setVelocity(0);
            intake.intakeStop();
        } else {
            // 4. Shooter Spin-up & Shooting Logic
            if (gamepad1.right_bumper) {
                // Keep shooter spun up to currentRPM
                shooterMotor.setVelocity(currentRPM);

                // Right trigger fires: feed balls into flywheel with shootArtifacts
                if (gamepad1.right_trigger > 0.5) {
                    if (isAtSpeed()) {
                        intake.shootArtifacts(); // intake 0.5 feeds into flywheel
                    }
                } else if (!gamepad1.left_bumper && gamepad1.left_trigger <= 0.5) {
                    // RT not pressed and intake buttons not pressed -> stop intake
                    intake.intakeStop();
                }
            } else {
                // Not holding RB: spin down flywheel
                shooterMotor.setVelocity(0);
            }

            // 5. Intake Controls (when not actively shooting)
            if (gamepad1.left_bumper) {
                intake.startIntake();            // intake 0.75
            } else if (gamepad1.left_trigger > 0.5) {
                intake.startReverseIntake();     // intake -0.75
            } else if (!gamepad1.right_bumper || gamepad1.right_trigger <= 0.5) {
                // If neither intaking nor shooting, stop intake
                intake.intakeStop();
            }
        }

        // 6. Driver Telemetry Dashboard
        telemetry.addLine("=== SHOOTER STATUS ===");
        telemetry.addData("Target RPM", currentRPM);
        telemetry.addData("Actual Velocity", "%.1f RPM", shooterMotor.getVelocity());
        telemetry.addData("Flywheel At Speed?", isAtSpeed() ? "READY" : "SPINNING UP");
        telemetry.addData("Shooter Motor Power", "%.2f", shooterMotor.getPower());
        telemetry.addLine("=== INTAKE STATUS ===");
        telemetry.addData("Intake Motor Power", "%.2f", intake.intakeMotor.getPower());
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("LB: Intake | LT: Rev Intake | RB: Spin | RT: Shoot");
        telemetry.addLine("Y/A: RPM +/-25 | D-Pad: Presets | X: E-Stop");
        telemetry.update();
    }

    public boolean isAtSpeed() {
        return shooterMotor.getVelocity() >= currentRPM;
    }

    @Override
    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setVelocity(0);
        }
        if (intake != null) {
            intake.intakeStop();
        }
    }
}
