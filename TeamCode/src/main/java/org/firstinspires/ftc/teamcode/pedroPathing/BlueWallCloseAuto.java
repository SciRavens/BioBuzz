package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Blue Wall Close Auto", group = "Competition", preselectTeleOp = "RobotTeleop")
public class BlueWallCloseAuto extends OpMode {
    private static final PoseFactory P = PoseFactory.degrees();

    private final Pose startPose               = P.of(31, 135, 90);
    private final Pose scorePose               = P.of(58, 86, 143);
    private final Pose intakePose1Control1     = P.of(66, 80, 0);
    private final Pose intakePose1Control2     = P.of(52, 85, 0);
    private final Pose intakePose1             = P.of(12, 84, 180);

    private final Pose openGatePose            = P.of(133, 70, 180);
    private final Pose openGateControlPoint    = P.of(90, 76.5, 0);

    private final Pose intakePose2Control1     = P.of(45, 51, 0);
    private final Pose intakePose2Control2     = P.of(58, 61, 0);
    private final Pose intakePose2             = P.of(10, 60, 180);

    private final Pose intakePose3Control1     = P.of(41, 16, 0);
    private final Pose intakePose3Control2     = P.of(63, 39, 0);
    private final Pose intakePose3             = P.of(9, 36, 180);

    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Paths (Pedro Pathing 3)
    private Path scorePreload() { return line(startPose, scorePose).linear(startPose, scorePose); }
    private Path openGate()     { return curve(intakePose1, openGateControlPoint, openGatePose).constant(openGatePose); }
    private Path intakeStack1() { return curve(startPose, intakePose1Control1, intakePose1Control2, intakePose1).linear(startPose, intakePose1); }
    private Path scoreStack1()  { return line(intakePose1, scorePose).linear(intakePose1, scorePose); }
    private Path intakeStack2() { return curve(scorePose, intakePose2Control1, intakePose2Control2, intakePose2).linear(scorePose, intakePose2); }
    private Path scoreStack2()  { return line(intakePose2, scorePose).linear(intakePose2, scorePose); }
    private Path intakeStack3() { return curve(scorePose, intakePose3Control1, intakePose3Control2, intakePose3).linear(scorePose, intakePose3); }
    private Path scoreStack3()  { return line(intakePose3, scorePose).linear(intakePose3, scorePose); }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        robot = new Robot(hardwareMap, telemetry);
        Robot.updateAlliance("BLUE");
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        telemetry.addLine("Blue Wall Close Auto Initialized (Pedro Pathing 3)");
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
        telemetry.addData("mode", follower.mode());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        double elapsed = pathTimer.milliseconds();
        switch (pathState) {
            case 0:
                robot.shooter.startAutoCloseShoot();
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachedSpeed() || elapsed > 4000) {
                    robot.intake.shootArtifacts();
                    setPathState(2);
                }
                break;
            case 2:
                if (elapsed > 2000) {
                    robot.shooter.stopShoot();
                    robot.intake.startIntake();
                    setPathState(3);
                }
                break;
            case 3:
                if (elapsed > 2000) {
                    follower.follow(intakeStack1());
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() || elapsed > 4000) {
                    robot.shooter.startAutoCloseShoot();
                    follower.follow(scoreStack1());
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() || elapsed > 4000) {
                    robot.intake.shootArtifacts();
                    setPathState(6);
                }
                break;
            case 6:
                if (elapsed > 2000) {
                    robot.shooter.stopShoot();
                    robot.intake.startIntake();
                    setPathState(7);
                }
                break;
            case 7:
                if (elapsed > 2000) {
                    follower.follow(intakeStack2());
                    robot.shooter.startCloseShoot();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() || (elapsed > 4000 && robot.shooter.reachedSpeed())) {
                    follower.follow(scoreStack2());
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() || elapsed > 4000) {
                    robot.intake.shootArtifacts();
                    setPathState(10);
                }
                break;
            case 10:
                if (elapsed > 2000) {
                    robot.shooter.stopShoot();
                    robot.intake.startIntake();
                    setPathState(11);
                }
                break;
            case 11:
                if (elapsed > 2000) {
                    follower.follow(intakeStack3());
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy() || elapsed > 4000) {
                    robot.shooter.startCloseShoot();
                    follower.follow(scoreStack3());
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() || (elapsed > 4000 && robot.shooter.reachedSpeed())) {
                    robot.intake.shootArtifacts();
                    setPathState(14);
                }
                break;
            case 14:
                if (elapsed > 4000) {
                    robot.shooter.stopShoot();
                    robot.intake.stopIntake();
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
