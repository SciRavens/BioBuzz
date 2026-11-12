package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Map;
import java.util.TreeMap;

public class Shooter {

    public final int shooterCloseRPM = 970; // 950
    public final int shooterFarRPM = 1375;
    public final int shooterClosePassiveRPM = 1225;
    public final int shooterFarPassiveRPM = 6000;
    public final int autonShooterFarRPM = 1370;
    public final int autonMidRPM = 1050;
    public final int autonMidRedRPM = 1250;
    public final int autonMidBlueRPM = 1400;
    public final int shooterMidRedRPM = 1215;
    public final int shooterMidBlueRPM = 1400;
    public final int shooterMidRPM = 975;
    public final int shooterHumanRPM = -1200;
    public final int shooterOffRPM = 0;
    public final int autoClose = 800;
    public final int autoCloseRed = 1000;
    public final int autoCloseBlue = 1110;

    private int currentRPM = 0;
    private static final TreeMap<Double, Double> SHOOTER_LUT = new TreeMap<>();

    static {
        // Close data points
        SHOOTER_LUT.put(60.0, 1025.0);
        SHOOTER_LUT.put(67.74, 1075.0);
        SHOOTER_LUT.put(71.0, 1085.0);
        SHOOTER_LUT.put(76.43, 1085.0);
        SHOOTER_LUT.put(84.0, 1100.0);
        SHOOTER_LUT.put(95.0, 1150.0);
        SHOOTER_LUT.put(101.0, 1150.0);
        SHOOTER_LUT.put(107.0, 1150.0);
        SHOOTER_LUT.put(111.25, 1200.0);
        SHOOTER_LUT.put(121.55, 1275.0);
        // Far data points
        SHOOTER_LUT.put(145.0, 1375.0);
        SHOOTER_LUT.put(152.0, 1400.0);
        SHOOTER_LUT.put(167.0, 1500.0);
    }

    public DcMotorEx shooterMotor;

    public Shooter(HardwareMap hardwareMap) {
        if (hardwareMap != null) {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    public void startCloseShoot() {
        currentRPM = shooterCloseRPM;
        shooterMotor.setVelocity(shooterCloseRPM);
    }

    public void startAutoCloseShoot() {
        currentRPM = autoClose;
        shooterMotor.setVelocity(autoClose);
    }

    public void startAutoCloseRedShoot() {
        currentRPM = autoCloseRed;
        shooterMotor.setVelocity(autoCloseRed);
    }

    public void startAutoCloseBlueShoot() {
        currentRPM = autoCloseBlue;
        shooterMotor.setVelocity(autoCloseBlue);
    }

    public void startClosePassiveShoot() {
        currentRPM = shooterClosePassiveRPM;
        shooterMotor.setVelocity(shooterClosePassiveRPM);
    }

    public void startFarPassiveShoot() {
        currentRPM = shooterFarPassiveRPM;
        shooterMotor.setVelocity(shooterFarPassiveRPM);
    }

    public int getRpmbyDistance(double distance) {
        if (SHOOTER_LUT.isEmpty()) return 0;
        if (SHOOTER_LUT.containsKey(distance)) {
            return SHOOTER_LUT.get(distance).intValue();
        }
        Map.Entry<Double, Double> lowEntry = SHOOTER_LUT.floorEntry(distance);
        Map.Entry<Double, Double> highEntry = SHOOTER_LUT.ceilingEntry(distance);

        if (lowEntry == null) return highEntry.getValue().intValue();
        if (highEntry == null) return lowEntry.getValue().intValue();

        double x1 = lowEntry.getKey();
        double y1 = lowEntry.getValue();
        double x2 = highEntry.getKey();
        double y2 = highEntry.getValue();

        double interpolatedRPM = y1 + (distance - x1) * ((y2 - y1) / (x2 - x1));
        return (int) interpolatedRPM;
    }

    public void startShooterbyDistance(double distance) {
        int rpm = getRpmbyDistance(distance);
        currentRPM = rpm;
        shooterMotor.setVelocity(rpm);
    }

    public void startShooterByDistance(double distance) {
        startShooterbyDistance(distance);
    }

    public void setRPM(int newRPM) {
        this.currentRPM = newRPM;
        shooterMotor.setVelocity(newRPM);
    }

    public int getCurrentRPM() {
        return currentRPM;
    }

    public boolean isSafeToContinueShooting(double distance) {
        double dynThreshold = 20.0;
        if (distance < 130.0) {
            dynThreshold += 20.0;
        }
        return Math.abs(shooterMotor.getVelocity() - currentRPM) <= dynThreshold;
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
        } else {
        }
    }

    public void startReverseShoot() {
        shooterMotor.setVelocity(-shooterCloseRPM);
    }

    public void stopFlyWheel() {
        shooterMotor.setVelocity(0);
    }
}
