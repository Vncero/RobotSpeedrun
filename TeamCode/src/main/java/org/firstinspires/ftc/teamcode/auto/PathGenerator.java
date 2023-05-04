package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.actions.InterruptAction;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Units;

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

    public static Path generatePath(AutoParkPosition parkPosition, Side side) {
        Path p = new Path();
        p.add(generalWaypoint(new Pose2d( // initial position
                new Translation2d(Constants.Drivetrain.realWheelbase / 2, Units.tilesToMeters(-3 * side.getMultiplier())),
                new Rotation2d()
        )));

        p.add(generalWaypoint(new Pose2d( // transition to cone
                new Translation2d(Units.tilesToMeters(2.5), Units.tilesToMeters(-3 * side.getMultiplier())),
                Rotation2d.fromDegrees(90 * side.getMultiplier())
        )));

        for (int i = 0; i < 3; i++) {

            p.add(generalWaypoint(new Pose2d( // going to get cone
                    new Translation2d(Units.tilesToMeters(2.5), -Constants.Drivetrain.realWheelbase / 2 * side.getMultiplier()),
                    Rotation2d.fromDegrees(90 * side.getMultiplier())
            )));

            p.add(interruptWaypoint(new Pose2d( // get cone
                    new Translation2d(Units.tilesToMeters(2.5), -Constants.Drivetrain.realWheelbase / 2 * side.getMultiplier()),
                    Rotation2d.fromDegrees(90 * side.getMultiplier())
            ), () -> {
                // get cone
            }));

            p.add(generalWaypoint(new Pose2d( // going to score (high)
                    new Translation2d(Units.tilesToMeters(2.75), -Units.tilesToMeters(1.75) * side.getMultiplier()),
                    Rotation2d.fromDegrees(-45 * side.getMultiplier())
            )));

            p.add(interruptWaypoint(new Pose2d( // scoring (high)
                    new Translation2d(Units.tilesToMeters(2.75), -Units.tilesToMeters(1.75) * side.getMultiplier()),
                    Rotation2d.fromDegrees(-45 * side.getMultiplier())
            ), () -> {
                // score high
            }));
        }

        switch (parkPosition) {
            case ONE:
                // TODO:  program this; its slightly harder to use multipliers
        }

        return p;
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
