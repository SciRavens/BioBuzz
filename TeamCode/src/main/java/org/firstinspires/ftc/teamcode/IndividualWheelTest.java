package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Dedicated test OpMode strictly for testing the Right Rear Wheel.
 * Does not initialize or actuate any other motors, servos, or subsystems.
 */
@TeleOp(name = "Individual Wheel Test", group = "Test")
public class IndividualWheelTest extends OpMode {

    public static final String MOTOR_NAME = "rightRear";
    public static final String FALLBACK_NAME = "backRear";

    private DcMotorEx rightRearMotor;
    private String activeName = MOTOR_NAME;
    private boolean motorFound = false;

    private static final double DEAD_ZONE = 0.05;

    @Override
    public void init() {
        // Attempt to find the right rear motor from hardware configuration
        try {
            rightRearMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
            activeName = MOTOR_NAME;
            motorFound = true;
        } catch (Exception e1) {
            try {
                rightRearMotor = hardwareMap.get(DcMotorEx.class, FALLBACK_NAME);
                activeName = FALLBACK_NAME;
                motorFound = true;
            } catch (Exception e2) {
                motorFound = false;
            }
        }

        if (motorFound) {
            // Match the standard drivetrain direction (REVERSE for right side in Pedro Mecanum config)
            rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addLine("=== RIGHT REAR WHEEL TEST INITIALIZED ===");
            telemetry.addData("Motor Config Name", activeName);
            telemetry.addData("Direction", rightRearMotor.getDirection());
            telemetry.addLine("Status: READY");
        } else {
            telemetry.addLine("=== ERROR: MOTOR NOT FOUND ===");
            telemetry.addLine("Could not find '" + MOTOR_NAME + "' or '" + FALLBACK_NAME + "' in hardwareMap.");
            telemetry.addLine("Please verify your Robot Controller configuration.");
        }

        telemetry.addLine();
        telemetry.addLine("--- CONTROLS (Gamepad 1) ---");
        telemetry.addLine("  Left Stick Y    : Variable Throttle (Forward / Reverse)");
        telemetry.addLine("  Right Trigger   : Variable Forward Power");
        telemetry.addLine("  Left Trigger    : Variable Reverse Power");
        telemetry.addLine("  A               : Run Forward (50% Power)");
        telemetry.addLine("  Y               : Run Forward (100% Power)");
        telemetry.addLine("  B               : Run Reverse (50% Power)");
        telemetry.addLine("  X               : Emergency STOP (0% Power)");
        telemetry.addLine("  D-Pad Left      : Reset Encoder Position");
        telemetry.addLine("  D-Pad Up / Down : Set Direction (REVERSE / FORWARD)");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!motorFound || rightRearMotor == null) {
            telemetry.addLine("ERROR: Right rear motor is not connected or configured.");
            telemetry.addLine("Expected config name: '" + MOTOR_NAME + "' (or '" + FALLBACK_NAME + "')");
            telemetry.update();
            return;
        }

        // Direction manual override / toggle
        if (gamepad1.dpad_up) {
            rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (gamepad1.dpad_down) {
            rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // Encoder Reset
        if (gamepad1.dpad_left) {
            rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double power = 0.0;

        // 1. Stick control (-1.0 to 1.0)
        double stickY = -gamepad1.left_stick_y;
        if (Math.abs(stickY) > DEAD_ZONE) {
            power = stickY;
        }
        // 2. Trigger control (RT = forward, LT = reverse)
        else if (gamepad1.right_trigger > DEAD_ZONE) {
            power = gamepad1.right_trigger;
        } else if (gamepad1.left_trigger > DEAD_ZONE) {
            power = -gamepad1.left_trigger;
        }
        // 3. Button presets
        else if (gamepad1.y) {
            power = 1.0;
        } else if (gamepad1.a) {
            power = 0.5;
        } else if (gamepad1.b) {
            power = -0.5;
        } else if (gamepad1.x) {
            power = 0.0;
        }

        rightRearMotor.setPower(power);

        // Telemetry status
        telemetry.addLine("=== RIGHT REAR WHEEL TEST ===");
        telemetry.addData("Motor Config Name", activeName);
        telemetry.addData("Applied Power", "%.2f", power);
        telemetry.addData("Direction", rightRearMotor.getDirection());

        try {
            telemetry.addData("Encoder Position", rightRearMotor.getCurrentPosition());
            telemetry.addData("Velocity (ticks/s)", "%.1f", rightRearMotor.getVelocity());
        } catch (Exception ignored) {}

        try {
            double currentAmps = rightRearMotor.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("Current Draw", "%.2f A", currentAmps);
        } catch (Exception ignored) {}

        telemetry.update();
    }

    @Override
    public void stop() {
        if (rightRearMotor != null) {
            rightRearMotor.setPower(0);
        }
    }
}
