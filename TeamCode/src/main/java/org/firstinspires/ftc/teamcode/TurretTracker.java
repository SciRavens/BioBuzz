package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class TurretTracker {

    public static class FieldTarget {
        public final String name;
        public final int tagId;
        public final Pose goalCoordinates;

        public FieldTarget(String name, int tagId, double x, double y) {
            this.name = name;
            this.tagId = tagId;
            this.goalCoordinates = new Pose(x, y, 0);
        }

        public FieldTarget(String name, int tagId, int pipelineId, double x, double y) {
            this(name, tagId, x, y);
        }
    }

    // Default target configurations for the new season (easily updated as needed)
    public static final FieldTarget DEFAULT_RED_TARGET = new FieldTarget("RED", 24, 72.0, 96.0);
    public static final FieldTarget DEFAULT_BLUE_TARGET = new FieldTarget("BLUE", 20, 72.0, 48.0);

    // Field Center and boundary geometry
    public static final double FIELD_CENTER_X = 72.0;
    public static final double FIELD_CENTER_Y = 72.0;
    public static final double HYSTERESIS_DEADBAND = 6.0; // +/- 6 inches deadband around X=72

    private final Robot robot;

    private FieldTarget redTarget = DEFAULT_RED_TARGET;
    private FieldTarget blueTarget = DEFAULT_BLUE_TARGET;
    private FieldTarget activeTarget = DEFAULT_RED_TARGET;

    private long lastLoopTime = 0;
    private boolean autoSwitchEnabled = true;

    // Manual override state
    private boolean manualOverrideActive = false;
    private long manualReleaseTime = 0;
    private static final long MANUAL_SETTLE_DELAY_MS = 250; // Settle delay before re-engaging auto

    public TurretTracker(Robot robot) {
        this.robot = robot;
    }

    public void start() {
        lastLoopTime = System.nanoTime();
    }

    public void configureTargets(FieldTarget redTarget, FieldTarget blueTarget) {
        this.redTarget = redTarget;
        this.blueTarget = blueTarget;
    }

    public void setAutoSwitchEnabled(boolean enabled) {
        this.autoSwitchEnabled = enabled;
    }

    public void setActiveTarget(FieldTarget newTarget, Pose currentRobotPose) {
        if (newTarget == null || newTarget == this.activeTarget) return;

        this.activeTarget = newTarget;

        // Pre-orient the turret state toward the new target using Pedro 3 odometry
        double dx = newTarget.goalCoordinates.x() - currentRobotPose.x();
        double dy = newTarget.goalCoordinates.y() - currentRobotPose.y();
        double targetWorldAngle = Math.atan2(dy, dx);
        robot.turret.resetTrackingState(targetWorldAngle);
    }

    /**
     * Overloaded update for autonomous or default tracking without manual gamepad inputs.
     */
    public void update(Pose currentRobotPose) {
        update(currentRobotPose, false, false);
    }

    /**
     * Master update loop supporting manual button override, automatic field-side switching,
     * and odometry aiming.
     */
    public void update(Pose currentRobotPose, boolean manualLeft, boolean manualRight) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastLoopTime) / 1e9;
        lastLoopTime = currentTime;
        dt = Math.max(dt, 0.001); // Prevent division by zero

        // 1. Manual Button Override Check
        if (manualLeft && !manualRight) {
            manualOverrideActive = true;
            robot.turret.goLeft();
            reportTelemetry("🕹️ MANUAL (LEFT)", 0.0, -Turret.TURRET_MANUAL_POWER, currentRobotPose);
            return;
        } else if (manualRight && !manualLeft) {
            manualOverrideActive = true;
            robot.turret.goRight();
            reportTelemetry("🕹️ MANUAL (RIGHT)", 0.0, Turret.TURRET_MANUAL_POWER, currentRobotPose);
            return;
        } else if (manualOverrideActive) {
            // Driver just released manual buttons; start settle timer
            manualOverrideActive = false;
            manualReleaseTime = System.currentTimeMillis();
            robot.turret.stopTurret();
            reportTelemetry("⏳ SETTLING", 0.0, 0.0, currentRobotPose);
            return;
        }

        // Enforce brief settle delay after manual release
        if (System.currentTimeMillis() - manualReleaseTime < MANUAL_SETTLE_DELAY_MS) {
            robot.turret.stopTurret();
            reportTelemetry("⏳ SETTLING", 0.0, 0.0, currentRobotPose);
            return;
        }

        // 2. Automatic Field-Side Selection with Hysteresis (Centered at 72, 72)
        if (autoSwitchEnabled) {
            double currentX = currentRobotPose.x();
            if (currentX > FIELD_CENTER_X + HYSTERESIS_DEADBAND) {
                if (activeTarget != redTarget) {
                    setActiveTarget(redTarget, currentRobotPose);
                }
            } else if (currentX < FIELD_CENTER_X - HYSTERESIS_DEADBAND) {
                if (activeTarget != blueTarget) {
                    setActiveTarget(blueTarget, currentRobotPose);
                }
            }
            // Inside [72 - 6, 72 + 6]: keep current activeTarget without thrashing
        }

        // 3. Power Calculation: Odometry Aiming
        double turretPower = robot.turret.calculateOdometryPower(activeTarget.goalCoordinates, currentRobotPose, dt);
        String status = "🧠 ODOMETRY AIMING";

        robot.turret.setTurretPower(turretPower);
        reportTelemetry(status, robot.turret.getLastError(), turretPower, currentRobotPose);
    }

    private void reportTelemetry(String status, double error, double power, Pose robotPose) {
        if (robot.telemetry == null) return;
        robot.telemetry.addData("Active Target", "%s (Tag %d)", activeTarget.name, activeTarget.tagId);
        robot.telemetry.addData("Dist to Midline", "%.1f in", Math.abs(robotPose.x() - FIELD_CENTER_X));
        robot.telemetry.addData("Turret Status", status);
        robot.telemetry.addData("Turret Error", "%.2f°", error);
        robot.telemetry.addData("Turret Power", "%.3f", power);
    }

    public FieldTarget getActiveTarget() {
        return activeTarget;
    }
}
