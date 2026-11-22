package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.junit.Test;

import java.util.List;

import static org.junit.Assert.*;

public class FlywheelPIDFMetricsTest {

    @Test
    public void testRpmAndTicksPerSecConversionsFor6000RpmMotor() {
        assertEquals(6000.0, FlywheelPIDFMetrics.ticksPerSecToRpm(2800.0), 1e-6);
        assertEquals(3000.0, FlywheelPIDFMetrics.ticksPerSecToRpm(1400.0), 1e-6);
        assertEquals(2800.0, FlywheelPIDFMetrics.rpmToTicksPerSec(6000.0), 1e-6);
        assertEquals(1175.0, FlywheelPIDFMetrics.rpmToTicksPerSec(
                FlywheelPIDFMetrics.ticksPerSecToRpm(1175.0)), 1e-6);
    }

    @Test
    public void testFeedforwardAndRecommendedHeadroom() {
        double f = FlywheelPIDFMetrics.computeFeedforwardF(2800.0);
        assertEquals(32767.0 / 2800.0, f, 1e-6);

        assertEquals(0.0, FlywheelPIDFMetrics.computeFeedforwardF(0.0), 1e-6);
        assertEquals(0.0, FlywheelPIDFMetrics.computeFeedforwardF(-500.0), 1e-6);
        assertEquals(0.0, FlywheelPIDFMetrics.computeFeedforwardF(Double.NaN), 1e-6);

        assertEquals(2380.0, FlywheelPIDFMetrics.recommendedMaxTarget(2800.0), 1e-6);
        assertEquals(0.0, FlywheelPIDFMetrics.recommendedMaxTarget(-1.0), 1e-6);
    }

    @Test
    public void testMultiBallBurstTracksEachShotDipRecoveryAndOffset() {
        FlywheelPIDFMetrics.ShotTracker tracker =
                new FlywheelPIDFMetrics.ShotTracker(1175.0, 25.0, 45.0, 60.0);

        // Spin up and stabilize for >= 60ms -> ARMED
        tracker.update(0, 900.0, false);
        tracker.update(50, 1175.0, false);
        tracker.update(120, 1175.0, false);
        assertEquals(FlywheelPIDFMetrics.ShotTracker.State.ARMED, tracker.getState());

        // Ball #1: dips at t=200 to 1010, recovers into band at t=380, settles 60ms by t=440
        tracker.update(200, 1100.0, true);
        tracker.update(240, 1010.0, true);
        tracker.update(320, 1120.0, true);
        tracker.update(380, 1173.0, true);
        tracker.update(440, 1173.0, true);

        // Ball #2: brief overshoot to 1190? Test overshoot during recovery: dips at t=500, peaks 1210, settles t=700..760
        tracker.update(500, 1080.0, true);
        tracker.update(540, 1025.0, true);
        tracker.update(620, 1210.0, true); // overshoot above 1175+25=1200 resets settle
        tracker.update(700, 1174.0, true);
        tracker.update(760, 1174.0, true);

        // Ball #3: dips at t=820 to 1005, settles t=1040..1100
        tracker.update(820, 1090.0, true);
        tracker.update(860, 1005.0, true);
        tracker.update(1040, 1175.0, true);
        tracker.update(1100, 1175.0, true);

        tracker.finishBurst(1150, 1175.0);

        List<FlywheelPIDFMetrics.ShotRecord> shots = tracker.getShots();
        assertEquals(3, shots.size());
        assertEquals(0, tracker.getUnrecoveredCount());

        // Shot 1 checks
        assertEquals(1, shots.get(0).shotNumber);
        assertEquals(165.0, shots.get(0).dipDepth, 1e-6);
        assertEquals(180.0, shots.get(0).recoveryTimeMs, 1e-6);
        assertEquals(2.0, shots.get(0).steadyStateOffset, 1e-6);

        // Shot 2 checks (peak overshoot 1210 - 1175 = 35)
        assertEquals(2, shots.get(1).shotNumber);
        assertEquals(150.0, shots.get(1).dipDepth, 1e-6);
        assertEquals(200.0, shots.get(1).recoveryTimeMs, 1e-6);
        assertEquals(35.0, shots.get(1).peakOvershoot, 1e-6);
        assertEquals(1.0, shots.get(1).steadyStateOffset, 1e-6);

        // Shot 3 checks
        assertEquals(3, shots.get(2).shotNumber);
        assertEquals(170.0, shots.get(2).dipDepth, 1e-6);
        assertEquals(220.0, shots.get(2).recoveryTimeMs, 1e-6);
        assertEquals(0.0, shots.get(2).steadyStateOffset, 1e-6);

        assertEquals(170.0, tracker.getMaxDipDepth(), 1e-6);
        assertEquals(220.0, tracker.getWorstRecoveryMs(), 1e-6);
        assertEquals(2.0, tracker.getMaxAbsSteadyStateOffset(), 1e-6);
    }

    @Test
    public void testShotTrackerIgnoresNonFeedingDipAndHandlesHysteresisAndUnrecoveredOffset() {
        FlywheelPIDFMetrics.ShotTracker tracker =
                new FlywheelPIDFMetrics.ShotTracker(1200.0, 25.0, 50.0, 60.0);

        tracker.update(0, 1200.0, false);
        tracker.update(70, 1200.0, false);
        assertEquals(FlywheelPIDFMetrics.ShotTracker.State.ARMED, tracker.getState());

        // Dip while NOT feeding should not register a shot
        tracker.update(100, 1050.0, false);
        tracker.update(160, 1200.0, false);
        assertEquals(0, tracker.getShotCount());

        // Real shot with brief <60ms bounce into band that falls back below tolerance -> still 1 shot
        tracker.update(200, 1100.0, true);
        tracker.update(240, 1185.0, true); // inside band at 240
        tracker.update(270, 1130.0, true); // only 30ms later drops out -> stays same shot
        tracker.update(350, 1110.0, true); // remains offset below 1175 when burst ends

        tracker.finishBurst(400, 1110.0);

        assertEquals(1, tracker.getShotCount());
        assertEquals(1, tracker.getUnrecoveredCount());
        FlywheelPIDFMetrics.ShotRecord rec = tracker.getShots().get(0);
        assertFalse(rec.recovered());
        assertEquals(100.0, rec.dipDepth, 1e-6);
        assertEquals(90.0, rec.steadyStateOffset, 1e-6);
    }

    @Test
    public void testTrialCostPenalizesSteadyStateOffsetAndSearchStepperProgresses() {
        double wellTunedCost = FlywheelPIDFMetrics.computeTrialCost(1200.0, 50.0, 2.7, 1210.0, 4.0, 2.0);
        double offsetDroopCost = FlywheelPIDFMetrics.computeTrialCost(1200.0, 50.0, 2.7, 1210.0, 4.0, 80.0);
        assertTrue(wellTunedCost < offsetDroopCost);

        FlywheelPIDFMetrics.AutoSearchStepper stepper = new FlywheelPIDFMetrics.AutoSearchStepper();
        assertTrue(stepper.prepareNextCandidate());
        assertEquals("baseline", stepper.recordResult(10.0));

        assertTrue(stepper.prepareNextCandidate());
        double[] c1 = stepper.getCandidate();
        assertEquals(FlywheelPIDFMetrics.AutoSearchStepper.START[0] + FlywheelPIDFMetrics.AutoSearchStepper.START_STEP[0], c1[0], 1e-6);
        assertEquals("improved P", stepper.recordResult(7.0));
        assertEquals(c1[0], stepper.getBest()[0], 1e-6);
        assertEquals(7.0, stepper.getBestCost(), 1e-6);

        // Exhaust cleanly within MAX_TRIALS
        while (stepper.prepareNextCandidate()) {
            stepper.recordResult(8.0);
        }
        assertTrue(stepper.isFinished());
        assertTrue(stepper.getTrialCount() <= FlywheelPIDFMetrics.AutoSearchStepper.MAX_TRIALS);
    }

    @Test
    public void testShooterSharedConstantsAndNullSafety() {
        assertEquals(DcMotorSimple.Direction.FORWARD, Shooter.SHOOTER_DIRECTION);
        Shooter shooter = new Shooter(null);
        assertEquals(0.0, shooter.getVelocity(), 1e-6);
        assertNull(shooter.getPIDFCoefficients());
        shooter.setPIDFCoefficients(12.0, 0.5, 0.0, 11.7);
    }
}
