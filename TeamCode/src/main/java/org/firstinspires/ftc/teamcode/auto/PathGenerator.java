package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.actions.InterruptAction;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.FollowMecanumCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathsCommand;
import org.firstinspires.ftc.teamcode.commands.InterruptCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.ArrayList;
import java.util.List;


public class PathGenerator {
    private static Waypoint pointTurnWaypoint(Pose2d pose) {
        return new PointTurnWaypoint(pose,
                Constants.Trajectory.moveSpeed,
                Constants.Trajectory.turnSpeed,
                Constants.Trajectory.followRadius,
                Constants.Trajectory.positionBuffer,
                Constants.Trajectory.angleBuffer
        );
    }

    private static Waypoint startWaypoint(Pose2d pose) {
        return new StartWaypoint(pose);
    }

    private static Waypoint generalWaypoint(Pose2d pose) {
        return new GeneralWaypoint(pose,
                Constants.Trajectory.moveSpeed,
                Constants.Trajectory.turnSpeed,
                Constants.Trajectory.followRadius
        );
    }

    private static Waypoint interruptWaypoint(Pose2d pose, InterruptAction action) {
        return new InterruptWaypoint(pose,
                Constants.Trajectory.moveSpeed,
                Constants.Trajectory.turnSpeed,
                Constants.Trajectory.followRadius,
                Constants.Trajectory.positionBuffer,
                Constants.Trajectory.angleBuffer,
                action
        );
    }

    private static Waypoint endWaypoint(Pose2d pose) {
        return new EndWaypoint(pose,
                Constants.Trajectory.moveSpeed,
                Constants.Trajectory.turnSpeed,
                Constants.Trajectory.followRadius,
                Constants.Trajectory.positionBuffer,
                Constants.Trajectory.angleBuffer
        );
    }

    // pure pursuit fails, use this
    public static MecanumControllerCommand generateMecanumCommand(Drivetrain drivetrain, Trajectory trajectory, Pose2d desiredPose) {
        return new MecanumControllerCommand(
                trajectory,
                () -> desiredPose,
                Constants.Drivetrain.kinematics,
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI)),
        1.0,
                (speeds) -> {
                    drivetrain.getFrontLeft().motorEx.setVelocity(Units.metersToWheelTicks(speeds.frontLeftMetersPerSecond)); // TODO: figure out conversion from m/s to ticks/sec
                    drivetrain.getFrontRight().motorEx.setVelocity(Units.metersToWheelTicks(speeds.frontRightMetersPerSecond));
                    drivetrain.getBackLeft().motorEx.setVelocity(Units.metersToWheelTicks(speeds.rearLeftMetersPerSecond));
                    drivetrain.getBackRight().motorEx.setVelocity(Units.metersToWheelTicks(speeds.rearRightMetersPerSecond));
                }
        );
    }

    public static Command generatePursuitCommand(MecanumDrive drive, Odometry odometry, AutoParkPosition parkPosition, Side side) {
        return new FollowPathsCommand(drive,odometry, generatePath(parkPosition, side));
    }

    public static Command generateMecanumCommand(Drivetrain drivetrain, AutoParkPosition parkPosition, Side side) {
        return new FollowMecanumCommand(drivetrain, generateMecanumPaths(parkPosition, side));
    }

    public static List<Path> generatePath(AutoParkPosition parkPosition, Side side) {
        List<Path> paths = new ArrayList<>();
        Path p = new Path();
        paths.add(p);
        p.add(startWaypoint(new Pose2d( // initial position
                new Translation2d(Constants.Drivetrain.realWheelbase / 2, Units.tilesToMeters(-3 * side.getMultiplier())),
                new Rotation2d()
        )));

        p.add(generalWaypoint(new Pose2d( // transition to cone
                new Translation2d(Units.tilesToMeters(2.5), Units.tilesToMeters(-3 * side.getMultiplier())),
                Rotation2d.fromDegrees(90 * side.getMultiplier())
        )));

        p.add(endWaypoint(new Pose2d( // going to get cone
                new Translation2d(Units.tilesToMeters(2.5), -Constants.Drivetrain.realWheelbase / 2 * side.getMultiplier()),
                Rotation2d.fromDegrees(90 * side.getMultiplier())
        )));

        p.add(interruptWaypoint(new Pose2d(
                new Translation2d(),
                new Rotation2d()
        ), new InterruptCommand() {
            @Override
            public void execute() {

            }
        }));

//        for (int i = 0; i < 3; i++) {



//            p.add(interruptWaypoint(new Pose2d( // get cone
//                    new Translation2d(Units.tilesToMeters(2.5), -Constants.Drivetrain.realWheelbase / 2 * side.getMultiplier()),
//                    Rotation2d.fromDegrees(90 * side.getMultiplier())
//            ), () -> {
//                // get cone
//            }));
//
//            p.add(generalWaypoint(new Pose2d( // going to score (high)
//                    new Translation2d(Units.tilesToMeters(2.75), -Units.tilesToMeters(1.75) * side.getMultiplier()),
//                    Rotation2d.fromDegrees(-45 * side.getMultiplier())
//            )));
//
//            p.add(interruptWaypoint(new Pose2d( // scoring (high)
//                    new Translation2d(Units.tilesToMeters(2.75), -Units.tilesToMeters(1.75) * side.getMultiplier()),
//                    Rotation2d.fromDegrees(-45 * side.getMultiplier())
//            ), () -> {
//                // score high
//            }));
//        }
//        p.add(endWaypoint(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)))); // TODO: change to parkPosition

        switch (parkPosition) {
            case ONE:
                // TODO:  program this; its slightly harder to use multipliers
        }

        return paths;
    }

    public static List<MecanumControllerCommand> generateMecanumPaths(AutoParkPosition parkPosition, Side side) {
        return new ArrayList<>();
    }

    public static Trajectory generateTrajectory(Pose2d start, List<Translation2d> translations, Pose2d end, boolean reversed) {
        return TrajectoryGenerator.generateTrajectory(start, translations, end, new TrajectoryConfig(1, 0.5).setReversed(reversed));
    }

    public static MecanumControllerCommand generateMecanumCommand(Drivetrain drivetrain, Pose2d start, List<Translation2d> translations, Pose2d end, boolean reversed) {
        return new MecanumControllerCommand(
                generateTrajectory(start, translations, end, reversed),
                drivetrain::getPose,
                drivetrain.getKinematics(),
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI)),
                1.0,
                (speeds) -> {
                    drivetrain.getFrontLeft().motorEx.setVelocity(Units.metersToWheelTicks(speeds.frontLeftMetersPerSecond)); // TODO: figure out conversion from m/s to ticks/sec
                    drivetrain.getFrontRight().motorEx.setVelocity(Units.metersToWheelTicks(speeds.frontRightMetersPerSecond));
                    drivetrain.getBackLeft().motorEx.setVelocity(Units.metersToWheelTicks(speeds.rearLeftMetersPerSecond));
                    drivetrain.getBackRight().motorEx.setVelocity(Units.metersToWheelTicks(speeds.rearRightMetersPerSecond));
                }
        );
    }

    public enum AutoParkPosition {
        ONE,
        TWO,
        THREE
    }

    public enum Side {
        LEFT(1),
        RIGHT(-1);

        private final double multiplier;

        Side(double multiplier) {
            this.multiplier = multiplier;
        }

        public double getMultiplier() {
            return multiplier;
        }
    }
}
