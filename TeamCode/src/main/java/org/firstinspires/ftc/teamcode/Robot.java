package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public static final int BLUE_TARGET_TAG_ID = 20;
    public static final int PIPELINE_ID_BLUE = 8;
    public static final int RED_TARGET_TAG_ID = 24;
    public static final int PIPELINE_ID_RED = 2;

    public static final int RED_GOAL_X = 144;
    public static final int RED_GOAL_Y = 144;
    public static final int BLUE_GOAL_X = 0;
    public static final int BLUE_GOAL_Y = 144;

    public static String currentAlliance = "RED";
    public static int current_pipeline_id = PIPELINE_ID_RED;
    public static int current_tag_id = RED_TARGET_TAG_ID;
    public static int current_goal_x = RED_GOAL_X;
    public static int current_goal_y = RED_GOAL_Y;

    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public Gate gate;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    private Timer shootTimer;
    private boolean isAutonShootTimerOn = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.intake = new Intake(hardwareMap);
        this.shooter = new Shooter(hardwareMap);
        this.turret = new Turret(hardwareMap);
        this.gate = new Gate(hardwareMap, telemetry);
        this.shootTimer = new Timer();

        updateAlliance(currentAlliance);
    }

    public static void updateAlliance(String alliance) {
        currentAlliance = alliance;
        if ("BLUE".equalsIgnoreCase(alliance)) {
            current_pipeline_id = PIPELINE_ID_BLUE;
            current_tag_id = BLUE_TARGET_TAG_ID;
            current_goal_x = BLUE_GOAL_X;
            current_goal_y = BLUE_GOAL_Y;
        } else {
            current_pipeline_id = PIPELINE_ID_RED;
            current_tag_id = RED_TARGET_TAG_ID;
            current_goal_x = RED_GOAL_X;
            current_goal_y = RED_GOAL_Y;
        }
    }

    public double getDistanceFromGoal(Follower follower) {
        double dx = current_goal_x - follower.pose().x();
        double dy = current_goal_y - follower.pose().y();
        return Math.hypot(dx, dy);
    }

    public boolean autonShootByDistance(Follower follower, long timeoutMs, double distance) {
        gate.gateOpen();
        shooter.startShooterByDistance(distance);

        if (isAutonShootTimerOn) {
            if (shooter.isSafeToContinueShooting(distance)) {
                intake.feedBalls();
            } else {
                intake.stopFeeding();
            }
        } else if (shooter.reachedSpeed()) {
            shootTimer.reset();
            isAutonShootTimerOn = true;
            intake.feedBalls();
        } else {
            intake.stopFeeding();
        }

        if (isAutonShootTimerOn && shootTimer.milliseconds() > timeoutMs) {
            intake.stopFeeding();
            gate.gateClose();
            isAutonShootTimerOn = false;
            return true;
        }
        return false;
    }

    public boolean autonRapidShootByDistance(Follower follower, long timeoutMs, double distance) {
        gate.gateOpen();
        shooter.startAutoCloseBlueShoot();

        if (isAutonShootTimerOn) {
            intake.feedBalls();
        } else if (shooter.reachedSpeed()) {
            shootTimer.reset();
            isAutonShootTimerOn = true;
            intake.feedBalls();
        } else {
            intake.stopFeeding();
        }

        if (isAutonShootTimerOn && shootTimer.milliseconds() > timeoutMs) {
            intake.stopFeeding();
            gate.gateClose();
            isAutonShootTimerOn = false;
            return true;
        }
        return false;
    }

    public boolean autonShoot(Follower follower, long timeoutMs) {
        return autonShootByDistance(follower, timeoutMs, getDistanceFromGoal(follower));
    }

    public boolean autonRapidShoot(Follower follower, long timeoutMs) {
        return autonRapidShootByDistance(follower, timeoutMs, getDistanceFromGoal(follower));
    }

    public boolean TeleOpRapidShoot(Follower follower, long timeoutMs, double distance) {
        gate.gateOpen();
        shooter.startShooterByDistance(distance);
        if (shooter.reachedSpeed() && !isAutonShootTimerOn) {
            if (telemetry != null) telemetry.addLine("TeleOp Rapid Shoot On");
            shootTimer.reset();
            isAutonShootTimerOn = true;
            intake.feedBalls();
        }

        if (isAutonShootTimerOn && shootTimer.milliseconds() > timeoutMs) {
            if (telemetry != null) telemetry.addLine("TeleOp Rapid Shoot Off");
            intake.stopIntake();
            gate.gateClose();
            isAutonShootTimerOn = false;
            return true;
        }
        return false;
    }

    public boolean TeleOpShoot(Follower follower, long timeoutMs) {
        return TeleOpRapidShoot(follower, timeoutMs, getDistanceFromGoal(follower));
    }
}
