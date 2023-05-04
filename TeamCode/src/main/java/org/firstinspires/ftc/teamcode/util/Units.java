package org.firstinspires.ftc.teamcode.util;

public class Units {
    public static double inchesToMeters(double inches) {
        return inches / 39.37;
    }

    public static double metersToInches(double meters) {
        return meters * 39.37;
    }

    public static double tilesToMeters(double tiles) {
        return inchesToMeters(24 * tiles);
    }
}
