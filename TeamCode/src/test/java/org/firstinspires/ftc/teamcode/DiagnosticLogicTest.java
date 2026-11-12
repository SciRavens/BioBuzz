package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

import java.util.List;

public class DiagnosticLogicTest {

    @Test
    public void testDiagnosticModesEnum() {
        // Verify all 4 modes exist with correct labels
        DiagnosticTest.Mode[] modes = DiagnosticTest.Mode.values();
        assertEquals(4, modes.length);
        assertEquals("[1] DRIVETRAIN", DiagnosticTest.Mode.DRIVETRAIN.label);
        assertEquals("[2] SHOOTER", DiagnosticTest.Mode.SHOOTER.label);
        assertEquals("[3] INTAKE", DiagnosticTest.Mode.INTAKE.label);
        assertEquals("[4] FULL TELEOP", DiagnosticTest.Mode.TELEOP.label);
    }

    @Test
    public void testCargoTrackerQueueLogic() {
        VoltageCargoTracker tracker = new VoltageCargoTracker();

        // Initially empty
        assertEquals(0, tracker.getCount());
        assertTrue(tracker.isEmpty());
        assertFalse(tracker.isFull());

        // Add 4 items up to capacity
        assertTrue(tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN));
        assertEquals(1, tracker.getCount());
        assertEquals(1, tracker.getPollenCount());
        assertEquals(0, tracker.getNectarCount());

        assertTrue(tracker.addCargo(VoltageCargoTracker.CargoType.NECTAR));
        assertEquals(2, tracker.getCount());
        assertEquals(1, tracker.getPollenCount());
        assertEquals(1, tracker.getNectarCount());

        assertTrue(tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN));
        assertTrue(tracker.addCargo(VoltageCargoTracker.CargoType.NECTAR));

        // Now full (4/4)
        assertEquals(4, tracker.getCount());
        assertTrue(tracker.isFull());
        assertEquals(2, tracker.getPollenCount());
        assertEquals(2, tracker.getNectarCount());

        // 5th item should be rejected (capacity capped at 4)
        assertFalse(tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN));
        assertEquals(4, tracker.getCount());

        // Verify FIFO order: 1st in was POLLEN
        VoltageCargoTracker.CargoType popped1 = tracker.popShooterCargo();
        assertEquals(VoltageCargoTracker.CargoType.POLLEN, popped1);
        assertEquals(3, tracker.getCount());

        // 2nd in was NECTAR
        VoltageCargoTracker.CargoType popped2 = tracker.popShooterCargo();
        assertEquals(VoltageCargoTracker.CargoType.NECTAR, popped2);
        assertEquals(2, tracker.getCount());

        // Test clear
        tracker.clear();
        assertEquals(0, tracker.getCount());
        assertTrue(tracker.isEmpty());
    }

    @Test
    public void testCargoTrackerPopEmptyReturnsNull() {
        VoltageCargoTracker tracker = new VoltageCargoTracker();
        assertNull(tracker.popShooterCargo());
        assertNull(tracker.popEjectCargo());
    }

    @Test
    public void testCargoTrackerReverseEject() {
        VoltageCargoTracker tracker = new VoltageCargoTracker();
        tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN);
        tracker.addCargo(VoltageCargoTracker.CargoType.NECTAR);

        // popEjectCargo pops from the back (LIFO for intake ejection)
        VoltageCargoTracker.CargoType ejected = tracker.popEjectCargo();
        assertEquals(VoltageCargoTracker.CargoType.NECTAR, ejected);
        assertEquals(1, tracker.getCount());
    }

    @Test
    public void testCargoListCopySafety() {
        VoltageCargoTracker tracker = new VoltageCargoTracker();
        tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN);

        List<VoltageCargoTracker.CargoType> list = tracker.getCargoList();
        assertEquals(1, list.size());
        list.clear(); // mutating the returned list should not affect the internal queue

        assertEquals(1, tracker.getCount());
    }

    @Test
    public void testCargoFullCapacityAlert() {
        VoltageCargoTracker tracker = new VoltageCargoTracker();
        assertFalse(tracker.checkAndClearFullCapacityAlert());

        tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN);
        tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN);
        tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN);
        assertFalse(tracker.checkAndClearFullCapacityAlert());

        // Adding 4th ball hits capacity
        tracker.addCargo(VoltageCargoTracker.CargoType.POLLEN);
        assertTrue(tracker.checkAndClearFullCapacityAlert());

        // Subsequent check returns false (cleared)
        assertFalse(tracker.checkAndClearFullCapacityAlert());
    }

    @Test
    public void testShooterLUTInterpolation() {
        // Test Shooter LUT logic without requiring hardwareMap
        // We can instantiate Shooter with null hardwareMap if we mock or test pure math methods:
        // getRpmbyDistance logic is static data driven:
        // At 60.0 -> 1025
        // At 167.0 -> 1500
        // We can verify LUT bounds:
        // Close data points: 60.0 -> 1025, 84.0 -> 1100, 121.55 -> 1275
        // Far data points: 145.0 -> 1375, 167.0 -> 1500
        // Let's verify via the public fields on Shooter:
        Shooter shooter = new Shooter(null);

        assertEquals(1025, shooter.getRpmbyDistance(60.0));
        assertEquals(1100, shooter.getRpmbyDistance(84.0));
        assertEquals(1500, shooter.getRpmbyDistance(167.0));

        // Test below min distance clamps to lowest entry
        assertEquals(1025, shooter.getRpmbyDistance(50.0));

        // Test above max distance clamps to highest entry
        assertEquals(1500, shooter.getRpmbyDistance(200.0));

        // Test interpolation between 60.0 (1025) and 67.74 (1075)
        int midRPM = shooter.getRpmbyDistance(63.87);
        assertTrue(midRPM > 1025 && midRPM < 1075);
    }

    @Test
    public void testShooterPresetRPMValues() {
        Shooter shooter = new Shooter(null);
        assertEquals(970, shooter.shooterCloseRPM);
        assertEquals(1050, shooter.autonMidRPM);
        assertEquals(1375, shooter.shooterFarRPM);
    }
}
