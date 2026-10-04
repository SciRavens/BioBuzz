package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Gate subsystem for controlling ball/artifact release using gateServo.
 */
public class Gate {
    public Servo gateServo;
    public boolean gateClosed;

    public Gate(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            gateServo = hardwareMap.get(Servo.class, "gateServo");
        } catch (Exception e) {
            if (telemetry != null) {
                telemetry.addData("Warning", "gateServo not found in hardwareMap");
            }
        }
        gateClosed = true;
    }

    public void gateOpen() {
        if (gateServo != null && gateClosed) {
            gateServo.setPosition(0.025);
            gateClosed = false;
        }
    }

    public void gateClose() {
        if (gateServo != null && !gateClosed) {
            gateServo.setPosition(0.18);
            gateClosed = true;
        }
    }

    public boolean isGateClosed() {
        return gateClosed;
    }

    public boolean isGateOpen() {
        return !gateClosed;
    }
}
