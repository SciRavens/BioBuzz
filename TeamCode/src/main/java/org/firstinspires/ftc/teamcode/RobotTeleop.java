package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Standard Robot TeleOp for FTC using Pedro Pathing 3.
 * Handles robot driving, shooter motor control, intake, rapid fire sequences,
 * and voltage-based cargo tracking.
 */
@TeleOp(name = "RobotTeleop", group = "Competition")
public class RobotTeleop extends OpMode {

    private Timer timer;
    private Timer rapidTimer;
    private Timer shootIntervalTimer;

    private Follower follower;
    private Robot robot;

    private static final double DEAD_ZONE = 0.1;
    private boolean isRapidFireOn = false;

    @Override
    public void init() {
        timer = new Timer();
        rapidTimer = new Timer();
        shootIntervalTimer = new Timer();

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        // Alliance preselection
        if (gamepad1.y) {
            Robot.updateAlliance("BLUE");
        } else if (gamepad1.x) {
            Robot.updateAlliance("RED");
        }

        robot = new Robot(hardwareMap, telemetry);

        telemetry.addLine("RobotTeleop Initialized - Pedro Pathing 3");
        telemetry.addData("Alliance", Robot.currentAlliance);
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        // ------------------ ALLIANCE TOGGLE (IF CHANGED DURING RUN) ------------------
        if (gamepad1.y) {
            Robot.updateAlliance("BLUE");
        } else if (gamepad1.x) {
            Robot.updateAlliance("RED");
        }

        // ------------------ DRIVETRAIN (PEDRO 3 MANUAL DRIVE) ------------------
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        // In Pedro 3, manual drive powers are fed directly to follower.manual()
        follower.manual(yInput * powerScale, xInput * powerScale, turnInput * powerScale);
        follower.update();

        Pose currentPose = follower.pose();
        double distanceToGoal = robot.getDistanceFromGoal(follower);

        // ------------------ VOLTAGE CARGO TRACKER ------------------
        double drivetrainMagnitude = (Math.abs(xInput) + Math.abs(yInput) + Math.abs(turnInput)) * powerScale;
        robot.cargoTracker.update(drivetrainMagnitude);

        // Alert drivers when reaching maximum capacity (4/4 balls)
        if (robot.cargoTracker.checkAndClearFullCapacityAlert()) {
            gamepad1.rumble(300);
            gamepad2.rumble(300);
        }

        // Manual cargo queue reset override (D-Pad Down on Gamepad 2)
        if (gamepad2.dpad_down) {
            robot.cargoTracker.clear();
        }

        // ------------------ SHOOTER CONTROLS ------------------
        if (gamepad2.x) {
            robot.shooter.startFarShoot();
        } else if (gamepad2.b && !isRapidFireOn) {
            robot.shooter.startCloseShoot();
        } else if (gamepad1.a) {
            robot.shooter.startHumanIntake();
        } else if (gamepad2.a) {
            robot.shooter.stopShoot();
        }

        // ------------------ INTAKE CONTROLS ------------------
        if (gamepad2.left_bumper) {
            robot.intake.startIntakeOnly();
        } else if (gamepad2.left_trigger > 0.5) {
            robot.intake.startReverseIntake();
        } else if (!isRapidFireOn && !gamepad2.right_bumper) {
            robot.intake.stopIntake();
        }

        // ------------------ RAPID FIRE ------------------
        // Mid-Range Rapid Fire (gamepad2.y)
        if (gamepad2.y && !isRapidFireOn) {
            isRapidFireOn = true;
            robot.shooter.startMidShoot();
            rapidTimer.reset();
        }

        // Close-Range Rapid Fire (gamepad1.right_bumper or combined trigger)
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
            // Manual Trigger Shooting (gamepad2.right_bumper)
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
            } else {
                if (!gamepad2.left_bumper && gamepad2.left_trigger <= 0.5) {
                    robot.intake.stopIntake();
                }
            }
        }

        // ------------------ TELEMETRY ------------------
        robot.shooter.shooterLightUpdate();
        robot.cargoTracker.addTelemetry(telemetry);
        telemetry.addData("Alliance", Robot.currentAlliance);
        telemetry.addData("Rapid Fire Active", isRapidFireOn);
        telemetry.addData("Goal Distance", "%.1f in", distanceToGoal);
        telemetry.addData("Pose", "(%.1f, %.1f) @ %.1f°", currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.shooter.stopShoot();
        robot.intake.stopIntake();
        robot.cargoTracker.clear();
    }
}
