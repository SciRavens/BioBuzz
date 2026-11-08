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

/**
 * SquareTestAuto - Pedro Pathing 3 Sandbox OpMode
 *
 * Drives the robot in a closed square around the field and returns to the exact starting point.
 * Designed for tuning, testing, and experimenting with Pedro Pathing 3 features without
 * colliding with any field structures.
 */
@Autonomous(name = "Square Test Auto", group = "Test")
public class SquareTestAuto extends OpMode {

    public enum FieldPreset {
        QUADRANT_BOX,   // 36" x 36" in lower-left quadrant (clears HIVE by >12" and walls by >24")
        FULL_PERIMETER, // 96" x 96" perimeter circuit around the center HIVE (>23" clearance)
        CUSTOM          // Custom coordinates using CUSTOM_START_X, CUSTOM_START_Y, CUSTOM_SIDE_LENGTH
    }

    public enum HeadingMode {
        CONSTANT_STRAFE, // Robot maintains a fixed heading (0 deg) while strafing around the square
        TANGENT_FACING,  // Robot turns to face forward along each leg of the square
        SPINNING_LINEAR  // Robot smoothly rotates 90 deg along each leg
    }

    // =========================================================================
    // CONFIGURATION - Adjust these options to experiment with Pedro Pathing 3!
    // =========================================================================
    public static FieldPreset FIELD_PRESET = FieldPreset.QUADRANT_BOX;
    public static HeadingMode HEADING_MODE = HeadingMode.CONSTANT_STRAFE;

    public static double CUSTOM_START_X = 24.0;
    public static double CUSTOM_START_Y = 24.0;
    public static double CUSTOM_SIDE_LENGTH = 36.0;

    // Safety timeout per leg in milliseconds (transitions if endpoint is not reached)
    public static double LEG_TIMEOUT_MS = 5000.0;

    // =========================================================================
    // INTERNAL STATE
    // =========================================================================
    private static final PoseFactory P = PoseFactory.degrees();

    private Pose startPose;
    private Pose corner2, corner3, corner4;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState = 0;

    private Path leg1, leg2, leg3, leg4;

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        // 1. Resolve dimensions based on chosen preset
        double startX, startY, sideLength;
        switch (FIELD_PRESET) {
            case FULL_PERIMETER:
                startX = 24.0;
                startY = 24.0;
                sideLength = 96.0;
                break;
            case CUSTOM:
                startX = CUSTOM_START_X;
                startY = CUSTOM_START_Y;
                sideLength = CUSTOM_SIDE_LENGTH;
                break;
            case QUADRANT_BOX:
            default:
                startX = 24.0;
                startY = 24.0;
                sideLength = 36.0;
                break;
        }

        // 2. Define corner poses with respective headings
        startPose = P.of(startX, startY, 0);
        corner2   = P.of(startX + sideLength, startY, 90);
        corner3   = P.of(startX + sideLength, startY + sideLength, 180);
        corner4   = P.of(startX, startY + sideLength, 270);

        // 3. Construct paths using chosen heading mode
        buildPaths();

        // 4. Initialize Follower & set starting pose
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        telemetry.addLine("Square Test Auto Initialized (Pedro Pathing 3)");
        telemetry.addData("Preset", FIELD_PRESET);
        telemetry.addData("Heading Mode", HEADING_MODE);
        telemetry.addData("Square Dimensions", "%.1f\" x %.1f\"", sideLength, sideLength);
        telemetry.addData("Start Pose", "(%.1f, %.1f) @ 0°", startX, startY);
        telemetry.update();
    }

    private void buildPaths() {
        switch (HEADING_MODE) {
            case TANGENT_FACING:
                // Robot automatically rotates to face the direction of motion along each segment
                leg1 = line(startPose, corner2).tangent();
                leg2 = line(corner2, corner3).tangent();
                leg3 = line(corner3, corner4).tangent();
                leg4 = line(corner4, startPose).tangent();
                break;

            case SPINNING_LINEAR:
                // Robot smoothly interpolates its heading by 90 degrees across each segment
                leg1 = line(startPose, corner2).linear(startPose, corner2);
                leg2 = line(corner2, corner3).linear(corner2, corner3);
                leg3 = line(corner3, corner4).linear(corner3, corner4);
                leg4 = line(corner4, startPose).linear(corner4, startPose);
                break;

            case CONSTANT_STRAFE:
            default:
                // Robot maintains a constant 0 deg heading, translating forward and strafing laterally
                leg1 = line(startPose, corner2).constant(startPose);
                leg2 = line(corner2, corner3).constant(startPose);
                leg3 = line(corner3, corner4).constant(startPose);
                leg4 = line(corner4, startPose).constant(startPose);
                break;
        }
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

        // Real-time telemetry and diagnostics
        Pose currentPose = follower.pose();
        telemetry.addData("State", pathStateDescription(pathState));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Current Pose", "(%.2f, %.2f) @ %.1f°",
                currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
        telemetry.addData("Distance to End", "%.2f in", follower.distanceToEndpoint());
        telemetry.addData("Completion", "%.1f%%", follower.completion() * 100.0);
        telemetry.addData("Leg Timer", "%.1f s", pathTimer.milliseconds() / 1000.0);
        telemetry.addData("Total Run Time", "%.1f s", opmodeTimer.milliseconds() / 1000.0);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        double elapsed = pathTimer.milliseconds();

        switch (pathState) {
            case 0: // Start Leg 1 (Bottom Edge: Left -> Right)
                follower.follow(leg1);
                setPathState(1);
                break;

            case 1: // Follow Leg 1 -> Corner 2
                if (!follower.isBusy() || follower.atParametricEnd() || elapsed > LEG_TIMEOUT_MS) {
                    follower.follow(leg2);
                    setPathState(2);
                }
                break;

            case 2: // Follow Leg 2 -> Corner 3 (Right Edge: Bottom -> Top)
                if (!follower.isBusy() || follower.atParametricEnd() || elapsed > LEG_TIMEOUT_MS) {
                    follower.follow(leg3);
                    setPathState(3);
                }
                break;

            case 3: // Follow Leg 3 -> Corner 4 (Top Edge: Right -> Left)
                if (!follower.isBusy() || follower.atParametricEnd() || elapsed > LEG_TIMEOUT_MS) {
                    follower.follow(leg4);
                    setPathState(4);
                }
                break;

            case 4: // Follow Leg 4 -> Start Position (Left Edge: Top -> Bottom)
                if (!follower.isBusy() || follower.atParametricEnd() || elapsed > LEG_TIMEOUT_MS) {
                    setPathState(5);
                }
                break;

            case 5: // Square Completed: Hold at start position
                follower.hold(startPose);
                break;
        }
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.reset();
    }

    private String pathStateDescription(int state) {
        switch (state) {
            case 0: return "0: Initializing Leg 1";
            case 1: return "1: Driving Leg 1 (Bottom Edge: +X)";
            case 2: return "2: Driving Leg 2 (Right Edge: +Y)";
            case 3: return "3: Driving Leg 3 (Top Edge: -X)";
            case 4: return "4: Driving Leg 4 (Left Edge: -Y / Return)";
            case 5: return "5: Square Complete (Holding Start Pose)";
            default: return "Unknown (" + state + ")";
        }
    }
}
