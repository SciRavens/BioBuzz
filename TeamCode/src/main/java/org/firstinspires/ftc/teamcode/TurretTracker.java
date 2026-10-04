package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class TurretTracker {

    public static class FieldTarget {
        public final String name;
        public final int tagId;
        public final int pipelineId;
        public final Pose goalCoordinates;

        public FieldTarget(String name, int tagId, int pipelineId, double x, double y) {
            this.name = name;
            this.tagId = tagId;
            this.pipelineId = pipelineId;
            this.goalCoordinates = new Pose(x, y, 0);
        }
    }

    // Default target configurations for the new season (easily updated as needed)
    public static final FieldTarget DEFAULT_RED_TARGET = new FieldTarget("RED", 24, 2, 72.0, 96.0);
    public static final FieldTarget DEFAULT_BLUE_TARGET = new FieldTarget("BLUE", 20, 8, 72.0, 48.0);

    // Field Center and boundary geometry
    public static final double FIELD_CENTER_X = 72.0;
    public static final double FIELD_CENTER_Y = 72.0;
    public static final double HYSTERESIS_DEADBAND = 6.0; // +/- 6 inches deadband around X=72

    private final Robot robot;
    private final Limelight3A limelight;

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
        this.limelight = robot.hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start() {
        limelight.pipelineSwitch(activeTarget.pipelineId);
        limelight.start();
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
        limelight.pipelineSwitch(newTarget.pipelineId);

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
     * AprilTag underside angle deduction, and odometry fallback aiming.
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

        // 3. Vision Detection & Underside AprilTag 3D Angle Deduction
        LLResult result = limelight.getLatestResult();
        boolean trackingTag = false;
        double tx = 0.0;
        double tagYaw = 0.0;
        double tagPitch = 0.0;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == activeTarget.tagId) {
                        tx = fiducial.getTargetXDegrees();
                        trackingTag = true;

                        // Deduct 3D orientation of the underside AprilTag
                        Pose3D targetPose = fiducial.getTargetPoseCameraSpace();
                        if (targetPose != null && targetPose.getOrientation() != null) {
                            tagYaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);
                            tagPitch = targetPose.getOrientation().getPitch(AngleUnit.DEGREES);
                            // Detect HIVE tilt state (tipped toward vs tipped away)
                            robot.turret.deduceBasketState(tagPitch);
                        }
                        break;
                    }
                }
            }
        }

        // 4. Power Calculation: Angle-Deducted Vision PID vs. Odometry Fallback
        double turretPower;
        String status;

        if (trackingTag) {
            turretPower = robot.turret.calculateAngleDeductedPower(tx, tagYaw, true, currentRobotPose, dt);
            status = "🎯 VISION LOCKED";
        } else {
            // Target is occluded or outside camera FOV -> aim toward goal coordinates
            turretPower = robot.turret.calculateOdometryPower(activeTarget.goalCoordinates, currentRobotPose, dt);
            status = "🧠 ODOMETRY AIMING";
        }

        robot.turret.setTurretPower(turretPower);
        reportTelemetry(status, robot.turret.getLastError(), turretPower, currentRobotPose);
    }

    private void reportTelemetry(String status, double error, double power, Pose robotPose) {
        if (robot.telemetry == null) return;
        robot.telemetry.addData("Active Target", "%s (Tag %d, Pipe %d)", activeTarget.name, activeTarget.tagId, activeTarget.pipelineId);
        robot.telemetry.addData("Dist to Midline", "%.1f in", Math.abs(robotPose.x() - FIELD_CENTER_X));
        robot.telemetry.addData("Basket State", robot.turret.getLastBasketState());
        robot.telemetry.addData("Tag 3D Yaw/Pitch", "Yaw: %.1f°, Pitch: %.1f°", robot.turret.getLastTagYawDegrees(), robot.turret.getLastTagPitchDegrees());
        robot.telemetry.addData("Turret Status", status);
        robot.telemetry.addData("Turret Error", "%.2f°", error);
        robot.telemetry.addData("Turret Power", "%.3f", power);
    }

    public FieldTarget getActiveTarget() {
        return activeTarget;
    }
}
