package org.firstinspires.ftc.teamcode.pedroPathing;

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

@Autonomous(name = "Red Far Auto", group = "Competition", preselectTeleOp = "RobotTeleop")
public class RedFarAuto extends OpMode {
    private static final PoseFactory P = PoseFactory.degrees();

    // Field Poses (degrees via PoseFactory)
    private final Pose startPose           = P.of(98, 8, 0);
    private final Pose firstPattern        = P.of(131, 83.4, 0);
    private final Pose firstPatternPickUp  = P.of(144, 83.5, 0);
    private final Pose secondPattern       = P.of(98, 59.5, 0);
    private final Pose secondPatternPickUp = P.of(144, 59.5, 0);
    private final Pose thirdPattern        = P.of(98, 36, 0);
    private final Pose thirdPatternPickUp  = P.of(144, 36, 0);
    private final Pose shootingPose        = P.of(98, 8, 0);
    private final Pose finalPose           = P.of(120, 10, 0);

    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Paths (Pedro Pathing 3)
    private Path goToThirdPattern()  { return line(startPose, thirdPattern).linear(startPose, thirdPattern); }
    private Path getThirdPattern()   { return line(thirdPattern, thirdPatternPickUp).constant(thirdPatternPickUp); }
    private Path shootStack1()       { return line(thirdPatternPickUp, shootingPose).constant(shootingPose); }

    private Path goToSecondPattern() { return line(shootingPose, secondPattern).linear(shootingPose, secondPattern); }
    private Path getSecondPattern()  { return line(secondPattern, secondPatternPickUp).constant(secondPatternPickUp); }
    private Path shootStack2()       { return line(secondPatternPickUp, shootingPose).constant(shootingPose); }

    private Path goToFirstPattern()  { return line(shootingPose, firstPattern).linear(shootingPose, firstPattern); }
    private Path getFirstPattern()   { return line(firstPattern, firstPatternPickUp).constant(firstPatternPickUp); }
    private Path shootStack3()       { return line(firstPatternPickUp, shootingPose).constant(shootingPose); }

    private Path endingAuton()       { return line(shootingPose, finalPose).constant(shootingPose); }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        robot = new Robot(hardwareMap, telemetry);
        Robot.updateAlliance("RED");
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        telemetry.addLine("Red Far Auto Initialized (Pedro Pathing 3)");
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
                robot.shooter.startAutonFarShoot();
                setPathState(1);
                break;
            case 1:
                if (robot.shooter.reachedSpeed() || elapsed > 3000) {
                    robot.intake.shootArtifacts();
                    setPathState(2);
                }
                break;
            case 2:
                if (elapsed > 3000) {
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopIntake();
                    follower.follow(goToThirdPattern());
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy() || elapsed > 1000) {
                    follower.follow(getThirdPattern());
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() || elapsed > 1500) {
                    follower.follow(shootStack1());
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() || elapsed > 1000) {
                    robot.shooter.startAutonFarShoot();
                    setPathState(5);
                }
                break;
            case 5:
                if (robot.shooter.reachedSpeed() || elapsed > 3000) {
                    robot.intake.shootArtifacts();
                    setPathState(6);
                }
                break;
            case 6:
                if (elapsed > 3000) {
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopIntake();
                    follower.follow(goToSecondPattern());
                    setPathState(61);
                }
                break;
            case 61:
                if (!follower.isBusy() || elapsed > 1000) {
                    follower.follow(getSecondPattern());
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() || elapsed > 2000) {
                    follower.follow(shootStack2());
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() || elapsed > 2500) {
                    robot.shooter.startAutonFarShoot();
                    setPathState(9);
                }
                break;
            case 9:
                if (robot.shooter.reachedSpeed() || elapsed > 3000) {
                    robot.intake.shootArtifacts();
                    setPathState(10);
                }
                break;
            case 10:
                if (elapsed > 2500) {
                    robot.shooter.stopFlyWheel();
                    robot.intake.stopIntake();
                    follower.follow(endingAuton());
                    setPathState(11);
                }
                break;
            case 11:
                if (elapsed > 1000) {
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
