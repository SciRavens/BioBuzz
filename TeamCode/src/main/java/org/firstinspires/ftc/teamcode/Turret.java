package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Subsystem controlling the robot's rotating turret mechanism.
 * Uses a DcMotorEx configured with BRAKE zero power behavior and RUN_WITHOUT_ENCODER mode.
 */
public class Turret {

    public static final double DEFAULT_TURRET_POWER = 0.50;

    public DcMotorEx turretMotor;
    private double currentPower = 0.0;

    public Turret(HardwareMap hardwareMap) {
        if (hardwareMap != null) {
            try {
                turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
                turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (Exception e) {
                turretMotor = null;
            }
        }
    }

    /**
     * Turns the turret clockwise / right at DEFAULT_TURRET_POWER.
     */
    public void turnClockwise() {
        turn(DEFAULT_TURRET_POWER);
    }

    /**
     * Turns the turret counter-clockwise / left at DEFAULT_TURRET_POWER.
     */
    public void turnCounterClockwise() {
        turn(-DEFAULT_TURRET_POWER);
    }

    /**
     * Sets the turret motor power with clipping to [-1.0, 1.0].
     *
     * @param power Power to apply to turret motor.
     */
    public void turn(double power) {
        currentPower = Math.max(-1.0, Math.min(1.0, power));
        if (turretMotor != null) {
            turretMotor.setPower(currentPower);
        }
    }

    /**
     * Halts turret motor rotation and engages BRAKE mode.
     */
    public void stop() {
        currentPower = 0.0;
        if (turretMotor != null) {
            turretMotor.setPower(0.0);
        }
    }

    /**
     * Returns the currently applied power setting.
     */
    public double getPower() {
        if (turretMotor != null) {
            return turretMotor.getPower();
        }
        return currentPower;
    }

    /**
     * Returns the current encoder velocity in ticks per second (if encoder attached).
     */
    public double getVelocity() {
        if (turretMotor != null) {
            try {
                return turretMotor.getVelocity();
            } catch (Exception ignored) {
                return 0.0;
            }
        }
        return 0.0;
    }

    /**
     * Returns the current encoder tick position (if encoder attached).
     */
    public int getCurrentPosition() {
        if (turretMotor != null) {
            try {
                return turretMotor.getCurrentPosition();
            } catch (Exception ignored) {
                return 0;
            }
        }
        return 0;
    }

    /**
     * Returns the current motor electrical current in Amperes.
     */
    public double getCurrentAmps() {
        if (turretMotor != null) {
            try {
                return turretMotor.getCurrent(CurrentUnit.AMPS);
            } catch (Exception ignored) {
                return 0.0;
            }
        }
        return 0.0;
    }
}
