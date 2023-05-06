package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.MecanumControllerCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.List;

public class FollowMecanumCommand extends SequentialCommandGroup {
    public FollowMecanumCommand(Drivetrain drivetrain, List<MecanumControllerCommand> trajectories) {
//        addCommands(new InstantCommand(() -> drivetrain.resetOdometry(trajectories.get(0).getInitialPose())));
//        for (Trajectory trajectory : trajectories) {
//            addCommands();
//        }
    }
}
