package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Minimal Pedro Pathing 3 drivetrain manual driving sample.
 */
@TeleOp(name = "DriveTrainSample", group = "Examples")
public class DriveTrainSample extends OpMode {

    private Follower follower;
    private static final double DEAD_ZONE = 0.1;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        telemetry.addLine("DriveTrainSample Initialized (Pedro Pathing 3)");
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? -gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? -gamepad1.right_stick_x : 0;

        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.manual(yInput * powerScale, xInput * powerScale, turnInput * powerScale);
        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("Drive X", xInput);
        telemetry.addData("Drive Y", yInput);
        telemetry.addData("Turn", turnInput);
        telemetry.addData("Pose X", "%.2f", pose.x());
        telemetry.addData("Pose Y", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f°", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
