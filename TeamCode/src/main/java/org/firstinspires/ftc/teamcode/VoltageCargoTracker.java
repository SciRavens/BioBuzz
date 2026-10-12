package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

/**
 * Voltage-based cargo tracking subsystem for the 2026–2027 FTC game BIOBUZZ™.
 *
 * Tracks the quantity and type (Pollen vs. Nectar) of balls stored inside the robot
 * using built-in REV Control Hub voltage telemetry without external hardware sensors.
 */
public class VoltageCargoTracker {

    public static final int MAX_CAPACITY = 4;
    public static double POLLEN_DIP_MIN = 0.15;
    public static double NECTAR_DIP_MIN = 0.35;
    public static double BASELINE_ALPHA = 0.98;
    public static double VALLEY_RISE_THRESHOLD = 0.08;
    public static double VALLEY_DROP_THRESHOLD = 0.10;
    public static double RECOVERY_RESET_THRESHOLD = 0.08;
    public static long MIN_BALL_INTERVAL_MS = 200;
    public static double INTAKE_POWER_THRESHOLD = 0.10;
    public static double DRIVETRAIN_SUPPRESSION_THRESHOLD = 0.60;

    public enum TrackerState {
        IDLE,
        IN_DIP,
        SADDLE_RECOVERY
    }

    private final HardwareMap hardwareMap;
    private final Intake intake;
    private final Deque<CargoType> cargoQueue = new ArrayDeque<>(MAX_CAPACITY);
    private TrackerState state = TrackerState.IDLE;

    private double baselineVoltage = -1.0;
    private double currentVoltage = 12.0;
    private double currentDip = 0.0;
    private double dipMinVoltage = 12.0;
    private double maxDipInEvent = 0.0;
    private double saddlePeakVoltage = 12.0;

    private long lastBallEventTimeMs = 0;
    private long saddleEntryTimeMs = 0;
    private boolean justReachedFull = false;

    private CargoType lastDetectedType = null;
    private double lastDetectedDip = 0.0;

    public VoltageCargoTracker(HardwareMap hardwareMap, Intake intake) {
        this.hardwareMap = hardwareMap;
        this.intake = intake;
    }

    public VoltageCargoTracker(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public VoltageCargoTracker() {
        this(null, null);
    }

    public void update() {
        update(0.0);
    }

    public void update(double drivetrainMagnitude) {
        double v = readHardwareVoltage();
        double p = getIntakePower();
        long now = System.currentTimeMillis();
        processVoltageSample(v, p, drivetrainMagnitude, now);
    }

    public synchronized void processVoltageSample(double voltage, double intakePower, double drivetrainMagnitude, long timeMs) {
        this.currentVoltage = voltage;
        if (baselineVoltage < 0) {
            baselineVoltage = voltage;
        }
        this.currentDip = Math.max(0.0, baselineVoltage - voltage);

        boolean isIntaking = intakePower > INTAKE_POWER_THRESHOLD;
        boolean isReversing = intakePower < -INTAKE_POWER_THRESHOLD;
        boolean isIntakeActive = isIntaking || isReversing;

        if (!isIntakeActive) {
            state = TrackerState.IDLE;
            if (currentDip < POLLEN_DIP_MIN) {
                updateBaseline(voltage);
            }
            return;
        }

        switch (state) {
            case IDLE:
                if (currentDip < POLLEN_DIP_MIN) {
                    updateBaseline(voltage);
                } else {
                    if (drivetrainMagnitude > DRIVETRAIN_SUPPRESSION_THRESHOLD) {
                        double intakeCurrent = getIntakeCurrentAmps();
                        if (intakeCurrent <= 0.0 || intakeCurrent < 2.0) {
                            return;
                        }
                    }
                    state = TrackerState.IN_DIP;
                    dipMinVoltage = voltage;
                    maxDipInEvent = currentDip;
                }
                break;

            case IN_DIP:
                if (voltage < dipMinVoltage) {
                    dipMinVoltage = voltage;
                }
                maxDipInEvent = Math.max(maxDipInEvent, baselineVoltage - voltage);

                double riseFromMin = voltage - dipMinVoltage;
                if (riseFromMin >= VALLEY_RISE_THRESHOLD) {
                    if (timeMs - lastBallEventTimeMs >= MIN_BALL_INTERVAL_MS) {
                        if (isIntaking) {
                            CargoType type = (maxDipInEvent >= NECTAR_DIP_MIN) ? CargoType.NECTAR : CargoType.POLLEN;
                            addCargo(type);
                            lastDetectedType = type;
                            lastDetectedDip = maxDipInEvent;
                            lastBallEventTimeMs = timeMs;
                        } else if (isReversing) {
                            CargoType ejected = popEjectCargo();
                            lastDetectedType = ejected;
                            lastDetectedDip = maxDipInEvent;
                            lastBallEventTimeMs = timeMs;
                        }
                    }
                    saddlePeakVoltage = voltage;
                    saddleEntryTimeMs = timeMs;
                    state = TrackerState.SADDLE_RECOVERY;
                }
                break;

            case SADDLE_RECOVERY:
                if (voltage > saddlePeakVoltage) {
                    saddlePeakVoltage = voltage;
                }
                double dropFromSaddle = saddlePeakVoltage - voltage;
                boolean significantDip = (baselineVoltage - voltage) >= POLLEN_DIP_MIN;

                if (dropFromSaddle >= VALLEY_DROP_THRESHOLD && significantDip) {
                    state = TrackerState.IN_DIP;
                    dipMinVoltage = voltage;
                    maxDipInEvent = baselineVoltage - voltage;
                } else if ((baselineVoltage - voltage) < RECOVERY_RESET_THRESHOLD) {
                    state = TrackerState.IDLE;
                } else if (timeMs - saddleEntryTimeMs > 800) {
                    state = TrackerState.IDLE;
                }
                break;
        }
    }

    private void updateBaseline(double voltage) {
        baselineVoltage = (BASELINE_ALPHA * baselineVoltage) + ((1.0 - BASELINE_ALPHA) * voltage);
    }

    public double readHardwareVoltage() {
        double minVoltage = Double.POSITIVE_INFINITY;
        if (hardwareMap != null && hardwareMap.voltageSensor != null) {
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double v = sensor.getVoltage();
                if (v > 0) {
                    minVoltage = Math.min(minVoltage, v);
                }
            }
        }
        return (minVoltage == Double.POSITIVE_INFINITY) ? (currentVoltage > 0 ? currentVoltage : 12.0) : minVoltage;
    }

    public double getIntakePower() {
        if (intake != null && intake.intakeMotor != null) {
            return intake.intakeMotor.getPower();
        }
        return 0.0;
    }

    public double getIntakeCurrentAmps() {
        if (intake != null && intake.intakeMotor != null) {
            try {
                return intake.intakeMotor.getCurrent(CurrentUnit.AMPS);
            } catch (Exception ignored) {}
        }
        return -1.0;
    }

    public synchronized boolean addCargo(CargoType type) {
        if (type == null) return false;
        if (cargoQueue.size() < MAX_CAPACITY) {
            cargoQueue.addLast(type);
            if (cargoQueue.size() == MAX_CAPACITY) {
                justReachedFull = true;
            }
            return true;
        }
        return false;
    }

    public synchronized CargoType popShooterCargo() {
        CargoType popped = cargoQueue.pollFirst();
        justReachedFull = false;
        return popped;
    }

    public synchronized CargoType popEjectCargo() {
        CargoType popped = cargoQueue.pollLast();
        justReachedFull = false;
        return popped;
    }

    public synchronized void clear() {
        cargoQueue.clear();
        justReachedFull = false;
    }

    public synchronized int getCount() {
        return cargoQueue.size();
    }

    public synchronized int getPollenCount() {
        int count = 0;
        for (CargoType c : cargoQueue) {
            if (c == CargoType.POLLEN) count++;
        }
        return count;
    }

    public synchronized int getNectarCount() {
        int count = 0;
        for (CargoType c : cargoQueue) {
            if (c == CargoType.NECTAR) count++;
        }
        return count;
    }

    public synchronized boolean isFull() {
        return cargoQueue.size() >= MAX_CAPACITY;
    }

    public synchronized boolean isEmpty() {
        return cargoQueue.isEmpty();
    }

    public synchronized List<CargoType> getCargoList() {
        return new ArrayList<>(cargoQueue);
    }

    public synchronized boolean checkAndClearFullCapacityAlert() {
        if (justReachedFull) {
            justReachedFull = false;
            return true;
        }
        return false;
    }

    public double getBatteryVoltage() { return currentVoltage; }
    public double getBaselineVoltage() { return baselineVoltage; }
    public double getCurrentDip() { return currentDip; }
    public TrackerState getState() { return state; }
    public CargoType getLastDetectedType() { return lastDetectedType; }
    public double getLastDetectedDip() { return lastDetectedDip; }
    public void setBaselineVoltage(double baselineVoltage) { this.baselineVoltage = baselineVoltage; }

    public synchronized void addTelemetry(Telemetry telemetry) {
        if (telemetry == null) return;
        int count = getCount();
        int pollen = getPollenCount();
        int nectar = getNectarCount();

        StringBuilder pips = new StringBuilder();
        for (int i = 0; i < MAX_CAPACITY; i++) {
            pips.append(i < count ? "●" : "○");
        }

        StringBuilder queueStr = new StringBuilder("[ ");
        if (cargoQueue.isEmpty()) {
            queueStr.append("EMPTY ");
        } else {
            for (CargoType c : cargoQueue) {
                queueStr.append(c.getCode()).append(" ");
            }
        }
        queueStr.append("] -> Shooter");

        String intakeState = "IDLE";
        double p = getIntakePower();
        if (p > INTAKE_POWER_THRESHOLD) intakeState = "INTAKING";
        else if (p < -INTAKE_POWER_THRESHOLD) intakeState = "REVERSING";

        telemetry.addLine("=== CARGO STATUS ===");
        telemetry.addData("Cargo Count", String.format(Locale.US, "[%s] %d/%d (Pollen: %d, Nectar: %d)",
                pips.toString(), count, MAX_CAPACITY, pollen, nectar));
        telemetry.addData("Queue (FIFO)", queueStr.toString());
        telemetry.addData("Battery", String.format(Locale.US, "%.2f V (Dip: %.2f V)", currentVoltage, currentDip));
        telemetry.addData("Intake State", intakeState);

        if (isFull()) {
            telemetry.addLine(">>> WARNING: INTAKE FULL (4/4) - HOARDING LIMIT <<<");
        }
    }
}
