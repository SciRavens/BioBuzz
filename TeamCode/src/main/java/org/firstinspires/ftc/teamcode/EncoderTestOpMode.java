package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Melonbotics absolute encoder test (analog output). The encoder's analog signal goes into an
 * analog input port on the Control Hub. Reports the absolute angle (0-360 deg) computed from
 * the output voltage as a fraction of the reference voltage.
 *
 * Config: add an Analog Input device on the port used, named "turretEncoder".
 *
 * If the reading is mirrored (angle decreases when it should increase), set REVERSED to true.
 * Use OFFSET_DEG to shift where zero is.
 */
@TeleOp(name = "Encoder Test", group = "Test")
public class EncoderTestOpMode extends OpMode {

    private static final String ENCODER_NAME = "turretEncoder";
    private static final boolean REVERSED = false;
    private static final double OFFSET_DEG = 0;

    private AnalogInput encoder;

    @Override
    public void init() {
        encoder = hardwareMap.get(AnalogInput.class, ENCODER_NAME);
        telemetry.addLine("Absolute encoder ready");
    }

    @Override
    public void loop() {
        double voltage = encoder.getVoltage();
        double angle = voltage / encoder.getMaxVoltage() * 360.0;
        if (REVERSED) angle = 360.0 - angle;
        angle = ((angle + OFFSET_DEG) % 360.0 + 360.0) % 360.0;

        telemetry.addData("Absolute angle (deg)", "%.2f", angle);
        telemetry.addData("Voltage", "%.3f / %.3f V", voltage, encoder.getMaxVoltage());
        telemetry.update();
    }
}
