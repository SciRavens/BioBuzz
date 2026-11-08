package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Utility OpMode for live testing and tuning Shooter RPM in increments of 25 RPM.
 */
@TeleOp(name = "Shooter RPM Test", group = "Test")
public class ShooterRPMTest extends OpMode {
    public int currentRPM = 500;

    public DcMotorEx shooterMotor;
    public RevBlinkinLedDriver shooterLight;

    public Robot robot;
    private Timer buttontimer;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        try {
            shooterLight = hardwareMap.get(RevBlinkinLedDriver.class, "shooterLight");
        } catch (Exception ignored) {}

        buttontimer = new Timer();
        buttontimer.reset();

        telemetry.addLine("Shooter RPM Test Initialized (Pedro Pathing 3)");
        telemetry.update();
    }

    @Override
    public void start() {
        buttontimer.reset();
    }

    @Override
    public void loop() {
        if (gamepad2.y && buttontimer.milliseconds() > 300) {
            currentRPM += 25;
            buttontimer.reset();
        } else if (gamepad2.a && buttontimer.milliseconds() > 300) {
            currentRPM -= 25;
            buttontimer.reset();
        }

        if (gamepad2.b) {
            shooterMotor.setVelocity(currentRPM);
        } else if (gamepad2.x) {
            shooterMotor.setVelocity(0);
        }

        if (shooterLight != null) {
            if (currentRPM > 0 && shooterMotor.getVelocity() >= currentRPM) {
                shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }

        if (gamepad2.dpad_up) {
            if (shooterMotor.getVelocity() >= currentRPM) {
                robot.intake.startIntakeOnly();
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }
        } else if (gamepad2.left_bumper) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopIntake();
        }

        telemetry.addData("Target RPM", currentRPM);
        telemetry.addData("Current Velocity", "%.1f", shooterMotor.getVelocity());
        telemetry.addData("Motor Power", "%.2f", shooterMotor.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setVelocity(0);
        if (robot != null) {
            robot.intake.stopIntake();
        }
    }
}
