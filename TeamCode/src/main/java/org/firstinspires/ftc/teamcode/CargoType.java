package org.firstinspires.ftc.teamcode;

/**
 * Represents the two spherical scoring elements in the 2026–2027 FTC game BIOBUZZ™:
 * - POLLEN: 2.8" diameter neutral ball.
 * - NECTAR: 3.6" diameter alliance ball (larger, denser).
 *
 * Robots are restricted to holding a maximum of 4 scoring elements at any time.
 */
public enum CargoType {
    POLLEN(2.8, "Pollen", 'P'),
    NECTAR(3.6, "Nectar", 'N');

    private final double diameterInches;
    private final String displayName;
    private final char code;

    CargoType(double diameterInches, String displayName, char code) {
        this.diameterInches = diameterInches;
        this.displayName = displayName;
        this.code = code;
    }

    public double getDiameterInches() {
        return diameterInches;
    }

    public String getDisplayName() {
        return displayName;
    }

    public char getCode() {
        return code;
    }

    @Override
    public String toString() {
        return displayName;
    }
}
