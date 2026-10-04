package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public CRServo turretCR;
    private static final double TURRET_POWER = 0.45;
    private static final double TURRET_STOP = 0.0;

    // Turret Control Constants
    private static final double TURRET_KP = 0.045;
    private static final double TURRET_KI = 0.002;
    private static final double TURRET_KD = 0.015;
    private static final double TURRET_DEADZONE = 0.3;
    private static final double TURRET_MAX_POWER = 0.7;
    private static final double TURRET_MIN_POWER = 0.05;
    private static final double TURRET_MAX_ACCELERATION = 1.5;
    private static final double FILTER_ALPHA = 0.7;
    private static final double INTEGRAL_LIMIT = 0.3;

    // PID State Variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastTurretPower = 0.0;
    private double filteredTx = 0.0;
    private double lastKnownTurretWorldAngle = 0.0;
    private Pose lastKnownRobotPose = new Pose(0, 0, 0);

    // Prediction/State Variables
    private boolean trackingLost = true;
    private double lastTx = 0.0;

    public Turret(HardwareMap hardwareMap) {
        turretCR = hardwareMap.get(CRServo.class, "turretServo");
        turretCR.setPower(0.0);
    }

    public void goLeft() {
        turretCR.setPower(-TURRET_POWER);
    }

    public void goRight() {
        turretCR.setPower(TURRET_POWER);
    }

    public void stopTurret() {
        turretCR.setPower(TURRET_STOP);
    }

    public void setTurretPower(double power) {
        turretCR.setPower(power);
    }

    public double calculatePower(double currentTx, boolean isTargetVisible, Pose currentRobotPose, double dt) {
        if (isTargetVisible) {
            filteredTx = FILTER_ALPHA * currentTx + (1 - FILTER_ALPHA) * filteredTx;
            lastKnownTurretWorldAngle = currentRobotPose.heading() + Math.toRadians(filteredTx);
            lastKnownRobotPose = currentRobotPose;
            trackingLost = false;
        } else {
            trackingLost = true;
            integralSum *= 0.95;
        }

        double error;
        if (!trackingLost) {
            error = filteredTx;
        } else {
            double predictedTurretAngle = lastKnownTurretWorldAngle - currentRobotPose.heading();
            error = Math.toDegrees(predictedTurretAngle);
        }

        double pTerm = TURRET_KP * error;
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        double iTerm = TURRET_KI * integralSum;

        double derivative = (error - lastError) / dt;
        double dTerm = TURRET_KD * derivative;

        double turretPower = pTerm + iTerm + dTerm;

        if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
            turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
        }

        turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

        double powerChange = turretPower - lastTurretPower;
        if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
            turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
        }

        if (Math.abs(error) < TURRET_DEADZONE) {
            turretPower = 0;
            if (!trackingLost) integralSum = 0;
        }

        lastError = error;
        lastTurretPower = turretPower;

        return turretPower;
    }

    public double calculateSimplePIDPower(double error, double dt) {
        double pTerm = TURRET_KP * error;
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
        double iTerm = TURRET_KI * integralSum;

        double derivative = (error - lastError) / dt;
        double dTerm = TURRET_KD * derivative;

        double turretPower = pTerm + iTerm + dTerm;

        if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
            turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
        }

        turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

        double powerChange = turretPower - lastTurretPower;
        if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
            turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
        }

        if (Math.abs(error) < TURRET_DEADZONE) {
            turretPower = 0;
            integralSum = 0;
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
}
