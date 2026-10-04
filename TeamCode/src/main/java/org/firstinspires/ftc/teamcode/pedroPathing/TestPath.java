package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "SampleAutoPush", group = "Examples")
public class TestPath extends OpMode {
    private static final PoseFactory P = PoseFactory.degrees();

    private final Pose scoreFarPose       = P.of(63, 8.5, 0);
    private final Pose pickupHumanPlayer  = P.of(9, 8.5, 90);

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Path scorePreload()    { return line(scoreFarPose, pickupHumanPlayer).linear(scoreFarPose, pickupHumanPlayer); }
    private Path grabHumanPlayer() { return line(pickupHumanPlayer, scoreFarPose).linear(pickupHumanPlayer, scoreFarPose); }
    private Path score()           { return line(scoreFarPose, pickupHumanPlayer).linear(scoreFarPose, pickupHumanPlayer); }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.create(hardwareMap);
        follower.setPose(scoreFarPose);

        telemetry.addLine("SampleAutoPush Initialized (Pedro Pathing 3)");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        Pose pose = follower.pose();
        telemetry.addData("path state", pathState);
        telemetry.addData("path timer (ms)", pathTimer.milliseconds());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.follow(grabHumanPlayer());
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.follow(score());
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(grabHumanPlayer());
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.follow(score());
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
