package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile.Constraints;

import org.firstinspires.ftc.teamcode.util.Units;

@Config
public class Constants {

    public static class Drivetrain {

        // TODO: do
        public static final double trackwidth = 0; // just the wheels
        public static final double wheelbase = 0;

        public static final double realTrackwidth = 0; // the actual dimensions of robot
        public static final double realWheelbase = 0; // the actual dimensions of robot

        public static final double ticksPerMotorRotation = 537.7;

        public static final double wheelDiameterMeters = Units.inchesToMeters(3.77953);

        public static final double gearReduction = 1;

        public static final double ticksPerMeter = (ticksPerMotorRotation * gearReduction) / (wheelDiameterMeters * Math.PI);
        public static final double metersPerTick = 1/ticksPerMeter;

        public static final Translation2d frontLeftWheelMeters = new Translation2d(trackwidth, wheelbase);
        public static final Translation2d frontRightWheelMeters = new Translation2d(trackwidth, -wheelbase);
        public static final Translation2d backLeftWheelMeters = new Translation2d(-trackwidth, wheelbase);
        public static final Translation2d backRightWheelMeters = new Translation2d(-trackwidth, -wheelbase);

        public static final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                frontLeftWheelMeters,
                frontRightWheelMeters,
                backLeftWheelMeters,
                backRightWheelMeters
        );
    }

    public static class LinearSlide {
        public static final double ticksPerMotorRotation = 537.7;

        public static final double spoolDiameter = 0.03225; // Units.inchesToMeters(1.75);

        public static final double gearReduction = 1;

        public static final double ticksPerMeter = (ticksPerMotorRotation * gearReduction) / (spoolDiameter * Math.PI);
        public static final double metersPerTick = 1/ticksPerMeter;

        public static double flipperKP = 0;

        public static double slideKP = 0.05;
        public static double slideKI = 0;
        public static double slideKD = 0;

        public static double maxVelocity = 1;
        public static double maxAcceleration = 0.5;

        public static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity ,maxAcceleration);

        public enum Position {
            HIGH(870, FlipperPosition.UP),
            MID(190, FlipperPosition.UP),
            LOW(1060, FlipperPosition.DOWN),
            GROUND(0, FlipperPosition.DOWN);

            private double slideHeightCentimeters;
            private FlipperPosition flipperPosition;

            Position(double slideHeightCentimeters, FlipperPosition flipperPosition) {
                this.slideHeightCentimeters = slideHeightCentimeters;
                this.flipperPosition = flipperPosition;
            }

            public double getSlideHeightCentimeters() {
                return this.slideHeightCentimeters;
            }
            public FlipperPosition getFlipperPosition() {
                return this.flipperPosition;
            }
        }

        public enum FlipperPosition {
            UP(269),
            DOWN(0);

            private double position;
            FlipperPosition(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    public static class Trajectory {
        public static final double followRadius = 0.5;
        public static final double positionBuffer = 0.25;
        public static final double angleBuffer = Math.toRadians(2.5);

        public static final double moveSpeed = 0.25;
        public static final double turnSpeed = 0.25;
    }
}