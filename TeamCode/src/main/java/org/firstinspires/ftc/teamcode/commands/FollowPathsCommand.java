package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.purepursuit.Path;

import java.util.List;

public class FollowPathsCommand extends CommandBase {

    private final MecanumDrive m_drive;
    private final Odometry m_odometry;
    private final List<Path> m_paths;

    private int pathIndex = 0;

    public FollowPathsCommand(MecanumDrive drive, Odometry odometry, List<Path> paths) {
        m_paths = paths;
        m_drive = drive;
        m_odometry = odometry;
    }

    @Override
    public void initialize() {
        for (Path p : m_paths) p.init();
    }

    public void addPath(Path p) {
        m_paths.add(p);
    }

    public void addWaypoints(Path... p) {
        for (Path path : p) this.addPath(path);
    }

    public void removePathAtIndex(int index) {
        m_paths.remove(index);
    }

    /**
     * Call this in a loop
     */
    @Override
    public void execute() {
        m_paths.get(pathIndex).followPath(m_drive, m_odometry);
        if (m_paths.get(pathIndex).isFinished()) pathIndex++;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return pathIndex == m_paths.size();
    }
}
