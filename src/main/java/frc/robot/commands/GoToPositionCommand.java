package frc.robot.commands;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;

/**
 * Command to drive to a target pose using on-the-fly PathPlanner path generation.
 */
public class GoToPositionCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;

    private Command followCommand;

    public GoToPositionCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        addRequirements(drivetrain);
    }

@Override
public void initialize() {
    Pose2d currentPose = drivetrain.getPose();


    followCommand.schedule();
}

    @Override
    public boolean isFinished() {
        return followCommand == null || followCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (followCommand != null) {
            followCommand.cancel();
        }
    }
}