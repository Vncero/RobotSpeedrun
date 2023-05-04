package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

public class Constants {

    public static class Drivebase {

        public static final double trackwidth = 0.5;
        public static final double wheelbase = 0.5;

        public static final double realTrackwidth = 0.6; // the actual dimensions of robot
        public static final double realWheelbase = 0.6;

        public static final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
                new Translation2d(trackwidth, wheelbase),
                new Translation2d(trackwidth, -wheelbase),
                new Translation2d(-trackwidth, wheelbase),
                new Translation2d(-trackwidth, -wheelbase)
        );
    }

    public static class Trajectory {
        public static final double followRadius = 0.5;
        public static final double positionBuffer = 0.25;
        public static final double angleBuffer = Math.toRadians(2.5);

        public static final double moveSpeed = 0.25;
        public static final double turnSpeed = 0.25;
    }
}
