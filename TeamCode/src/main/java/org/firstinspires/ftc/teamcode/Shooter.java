package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    public final int shooterCloseRPM = 970; // 950
    public final int shooterFarRPM = 1375;
    public final int autonShooterFarRPM = 1370;
    public final int autonMidRPM = 1050;
    public final int shooterMidRPM = 975;
    public final int shooterHumanRPM = -1200;
    public final int shooterOffRPM = 0;
    public final int autoClose = 800;

    private int currentRPM = 0;

    public DcMotorEx shooterMotor;
    public RevBlinkinLedDriver shooterLight;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLight = hardwareMap.get(RevBlinkinLedDriver.class, "shooterLight");
    }

    public void startCloseShoot() {
        currentRPM = shooterCloseRPM;
        shooterMotor.setVelocity(shooterCloseRPM);
    }

    public void startAutoCloseShoot() {
        currentRPM = autoClose;
        shooterMotor.setVelocity(autoClose);
    }

    public void startAutoMidShoot() {
        currentRPM = autonMidRPM;
        shooterMotor.setVelocity(autonMidRPM);
    }

    public void startAutonFarShoot() {
        currentRPM = autonShooterFarRPM;
        shooterMotor.setVelocity(autonShooterFarRPM);
    }

    public void startFarShoot() {
        currentRPM = shooterFarRPM;
        shooterMotor.setVelocity(shooterFarRPM);
    }

    public void startMidShoot() {
        currentRPM = shooterMidRPM;
        shooterMotor.setVelocity(shooterMidRPM);
    }

    public void startHumanIntake() {
        currentRPM = shooterHumanRPM;
        shooterMotor.setVelocity(shooterHumanRPM);
    }

    public void stopShoot() {
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        currentRPM = shooterOffRPM;
        shooterMotor.setVelocity(shooterOffRPM);
        shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public boolean reachedSpeed() {
        return shooterMotor.getVelocity() >= currentRPM;
    }

    public boolean reachMidSpeed() {
        return shooterMotor.getVelocity() >= shooterMidRPM;
    }

    public boolean reachAutoMidSpeed() {
        return shooterMotor.getVelocity() >= autonMidRPM;
    }

    public boolean reachCloseSpeed() {
        return shooterMotor.getVelocity() >= shooterCloseRPM;
    }

    public void shooterLightUpdate() {
        if (currentRPM == 0) {
            return;
        }
        if (reachedSpeed()) {
            shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    public void startReverseShoot() {
        shooterMotor.setVelocity(-shooterCloseRPM);
    }

    public void stopFlyWheel() {
        shooterMotor.setVelocity(0);
    }
}
