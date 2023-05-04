package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Constants;

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

    public static double wheelTicksToMeters(double ticks) {
        return ticks / Constants.Drivetrain.ticksPerMeter;
    }

    public static double metersToWheelTicks(double meters) {
        return meters * Constants.Drivetrain.ticksPerMeter;
    }

    public static double inchesToWheelTicks(double inches) {
        return metersToWheelTicks(inchesToMeters(inches));
    }
}
