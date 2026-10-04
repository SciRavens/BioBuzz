package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "RobotTeleop", group = "Competition")
public class RobotTeleop extends OpMode {

    private Timer timer;
    private Timer rapidTimer;

    private Follower follower;
    private Robot robot;
    private TurretTracker turretTracker;

    private static final double DEAD_ZONE = 0.1;
    private boolean isRapidFireOn = false;

    @Override
    public void init() {
        timer = new Timer();
        rapidTimer = new Timer();

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        robot = new Robot(hardwareMap, telemetry);
        turretTracker = new TurretTracker(robot);
        turretTracker.start();

        telemetry.addLine("RobotTeleop Initialized - Pedro Pathing 3 & TurretTracker");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        // ------------------ DRIVETRAIN (PEDRO 3 MANUAL DRIVE) ------------------
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        // In Pedro 3, manual drive powers are fed directly to follower.manual()
        follower.manual(yInput * powerScale, xInput * powerScale, turnInput * powerScale);
        follower.update();

        Pose currentPose = follower.pose();

        // ------------------ TURRET TRACKING & MANUAL OVERRIDE ------------------
        // Pressing D-Pad Left or Right overrides auto-tracking immediately
        turretTracker.update(currentPose, gamepad2.dpad_left, gamepad2.dpad_right);

        // ------------------ SHOOTER CONTROLS ------------------
        if (gamepad2.x) {
            robot.shooter.startFarShoot();
        } else if (gamepad2.b) {
            robot.shooter.startCloseShoot();
        } else if (gamepad1.a) {
            robot.shooter.startHumanIntake();
        } else if (gamepad2.a) {
            robot.shooter.stopShoot();
        }

        // ------------------ INTAKE CONTROLS ------------------
        if (gamepad2.left_bumper) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopIntake();
        }

        // ------------------ RAPID FIRE / TRANSFER ------------------
        if (gamepad2.y && !isRapidFireOn) {
            isRapidFireOn = true;
            robot.shooter.startMidShoot();
            rapidTimer.reset();
        }

        if (isRapidFireOn) {
            if (robot.shooter.reachedSpeed()) {
                robot.intake.shootArtifacts();
            }
            if (rapidTimer.milliseconds() >= 5750) {
                robot.shooter.stopFlyWheel();
                robot.intake.intakeStop();
                robot.intake.stopTransfer();
                isRapidFireOn = false;
            }
        } else {
            if (gamepad2.right_bumper) {
                if (robot.shooter.reachedSpeed()) {
                    robot.intake.startTransferOnly();
                }
            } else {
                robot.intake.stopTransfer();
            }
        }

        // ------------------ TELEMETRY ------------------
        robot.shooter.shooterLightUpdate();
        telemetry.addData("Pose", "(%.1f, %.1f) @ %.1f°", currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
        robot.intake.stopTransfer();
        robot.turret.stopTurret();
    }
}
