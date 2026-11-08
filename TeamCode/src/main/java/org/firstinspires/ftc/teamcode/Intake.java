package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotorEx intakeMotor;
    private final double INTAKE_POWER = 0.75;
    private final double SHOOTING_POWER = 0.5;
    private final double INTAKE_OFF = 0.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void startIntake() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void startIntakeOnly() {
        startIntake();
    }

    public void shootArtifacts() {
        intakeMotor.setPower(SHOOTING_POWER);
    }

    public void intakeStop() {
        intakeMotor.setPower(INTAKE_OFF);
    }

    public void stopIntake() {
        intakeStop();
    }

    public void feedBalls() {
        shootArtifacts();
    }

    public void stopFeeding() {
        intakeStop();
    }

    public void startReverseIntake() {
        intakeMotor.setPower(-INTAKE_POWER);
    }
}
