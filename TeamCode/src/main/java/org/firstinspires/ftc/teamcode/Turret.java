package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public enum BasketState {
        TIPPED_TOWARD,
        TIPPED_AWAY,
        UNKNOWN
    }

    public DcMotorEx turretCR;
    public static final double TURRET_MANUAL_POWER = 0.45;
    private static final double TURRET_STOP = 0.0;

    // Turret Control Constants (Tuned for smooth tracking)
    private static final double TURRET_KP = 0.045;
    private static final double TURRET_KI = 0.002;
    private static final double TURRET_KD = 0.015;
    private static final double TURRET_DEADZONE = 0.3; // Degrees
    private static final double TURRET_MAX_POWER = 0.7;
    private static final double TURRET_MIN_POWER = 0.05;
    private static final double TURRET_MAX_ACCELERATION = 1.5; // Max power change per second
    private static final double FILTER_ALPHA = 0.7; // Low-pass filter (0=history, 1=unfiltered)
    private static final double INTEGRAL_LIMIT = 0.3; // Anti-windup limit

    // PID State Variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastTurretPower = 0.0;
    private double filteredTx = 0.0;
    private double lastKnownTurretWorldAngle = 0.0; // Target angle in field coordinates (radians)
    private Pose lastKnownRobotPose = new Pose(0, 0, 0);

    // AprilTag 3D Angle & Basket State Variables
    private double lastTagYawDegrees = 0.0;
    private double lastTagPitchDegrees = 0.0;
    private BasketState lastBasketState = BasketState.UNKNOWN;
    private boolean trackingLost = true;

    public Turret(HardwareMap hardwareMap) {
        turretCR = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretCR.setPower(TURRET_STOP);
    }

    public void goLeft() {
        turretCR.setPower(-TURRET_MANUAL_POWER);
    }

    public void goRight() {
        turretCR.setPower(TURRET_MANUAL_POWER);
    }

    public void stopTurret() {
        turretCR.setPower(TURRET_STOP);
    }

    public void setTurretPower(double power) {
        turretCR.setPower(power);
    }

    /**
     * Resets filter and PID state on target switch to prevent abrupt jerks or residual windup.
     * Seeds the initial target world angle from odometry.
     */
    public void resetTrackingState(double initialWorldAngle) {
        lastError = 0.0;
        integralSum = 0.0;
        lastTurretPower = 0.0;
        filteredTx = 0.0;
        lastKnownTurretWorldAngle = initialWorldAngle;
        lastTagYawDegrees = 0.0;
        lastTagPitchDegrees = 0.0;
        lastBasketState = BasketState.UNKNOWN;
        trackingLost = true;
    }

    /**
     * Deducts basket tilt state from underside AprilTag pitch.
     * In Biobuzz, the HIVE is bi-stable (one CELL tilted up, one tilted down).
     * The underside tag pitch angle directly indicates whether the CELL is facing toward the robot.
     */
    public BasketState deduceBasketState(double pitchDegrees) {
        lastTagPitchDegrees = pitchDegrees;
        // Upward-facing CELL has underside pitched downward toward the floor
        if (pitchDegrees > 15.0) {
            lastBasketState = BasketState.TIPPED_TOWARD;
        } else if (pitchDegrees < -15.0) {
            lastBasketState = BasketState.TIPPED_AWAY;
        } else {
            lastBasketState = BasketState.UNKNOWN;
        }
        return lastBasketState;
    }

    /**
     * Deducts the physical angle of the turret on the robot chassis (degrees, 0 = forward),
     * combining the tag's known world bearing and the robot's odometry heading.
     */
    public double getDeducedTurretAngle(double robotHeadingRadians) {
        double relativeAngleRad = Angle.normalizeSigned(lastKnownTurretWorldAngle - robotHeadingRadians);
        return Math.toDegrees(relativeAngleRad);
    }

    /**
     * Calculates power using Vision error (tx) combined with angle-of-incidence compensation
     * from the underside AprilTag's 3D orientation.
     */
    public double calculateAngleDeductedPower(double currentTx, double tagYawDegrees, boolean isTargetVisible, Pose currentRobotPose, double dt) {
        lastTagYawDegrees = tagYawDegrees;

        // Angle-of-incidence compensation: minor adjustment to aim toward basket center
        // if viewing the underside tag at an acute angle
        double compensatedTx = currentTx;
        if (isTargetVisible && Math.abs(tagYawDegrees) > 10.0) {
            // Slight bias toward the normal vector into the basket
            compensatedTx -= (tagYawDegrees * 0.05);
        }

        return calculatePower(compensatedTx, isTargetVisible, currentRobotPose, dt);
    }

    /**
     * Standard vision tracking loop with odometry prediction when the tag is temporarily occluded.
     */
    public double calculatePower(double currentTx, boolean isTargetVisible, Pose currentRobotPose, double dt) {
        if (isTargetVisible) {
            filteredTx = FILTER_ALPHA * currentTx + (1.0 - FILTER_ALPHA) * filteredTx;
            lastKnownTurretWorldAngle = currentRobotPose.heading() + Math.toRadians(filteredTx);
            lastKnownRobotPose = currentRobotPose;
            trackingLost = false;
        } else {
            trackingLost = true;
            integralSum *= 0.95; // Decay integral to prevent windup while target is lost
        }

        double error;
        if (!trackingLost) {
            error = filteredTx;
        } else {
            // Prediction mode: keep aiming at the last known world angle
            double predictedTurretAngle = Angle.normalizeSigned(lastKnownTurretWorldAngle - currentRobotPose.heading());
            error = Math.toDegrees(predictedTurretAngle);
        }

        return executePID(error, dt, !trackingLost);
    }

    /**
     * Drives the turret toward given field coordinates using Pedro Pathing odometry.
     */
    public double calculateOdometryPower(Pose targetPose, Pose robotPose, double dt) {
        double deltaX = targetPose.x() - robotPose.x();
        double deltaY = targetPose.y() - robotPose.y();
        double absoluteTargetAngle = Math.atan2(deltaY, deltaX);

        // Normalize heading difference to [-180, 180] degrees
        double headingDiff = Angle.normalizeSigned(absoluteTargetAngle - robotPose.heading());
        double errorDegrees = Math.toDegrees(headingDiff);

        return executePID(errorDegrees, dt, false);
    }

    /**
     * Standalone PID execution for a direct error value.
     */
    public double calculateSimplePIDPower(double error, double dt) {
        return executePID(error, dt, false);
    }

    private double executePID(double error, double dt, boolean isVisionLocked) {
        // Proportional term
        double pTerm = TURRET_KP * error;

        // Integral term with anti-windup clamping
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        double iTerm = TURRET_KI * integralSum;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double dTerm = TURRET_KD * derivative;

        double turretPower = pTerm + iTerm + dTerm;

        // Apply minimum power threshold to overcome static friction
        if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
            turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
        }

        // Clamp to maximum allowed power
        turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

        // Velocity / acceleration rate limiting
        double powerChange = turretPower - lastTurretPower;
        if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
            turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
        }

        // Deadzone threshold for settling
        if (Math.abs(error) < TURRET_DEADZONE) {
            turretPower = 0.0;
            if (isVisionLocked) {
                integralSum = 0.0;
            }
        }

        lastError = error;
        lastTurretPower = turretPower;
        return turretPower;
    }

    public boolean isTrackingLost() {
        return trackingLost;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public double getLastError() {
        return lastError;
    }

    public double getLastTagYawDegrees() {
        return lastTagYawDegrees;
    }

    public double getLastTagPitchDegrees() {
        return lastTagPitchDegrees;
    }

    public BasketState getLastBasketState() {
        return lastBasketState;
    }
}
