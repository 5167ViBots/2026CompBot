// package frc.robot.commands;

// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.IdealStartingState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;
// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// import java.util.List;
// import java.util.function.BooleanSupplier;

// public class GoToPoseCommand extends Command {

//     private final CommandSwerveDrivetrain drivetrain;
//     private final Pose2d targetPose;
//     private Command followCommand;

//     public GoToPoseCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
//         this.drivetrain = drivetrain;
//         this.targetPose = targetPose;
//         addRequirements(drivetrain);
//     }

//     @Override
//     public void initialize() {
//         Pose2d currentPose = drivetrain.getPose();

//         // 1. Create waypoints (current → target)
//         List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
//             List.of(currentPose, targetPose)
//         );

//         // 2. Build path

//         PathPlannerPath path = new PathPlannerPath(waypoints, new PathConstraints(3.0, 3.0, 6, 6), null, new GoalEndState(0.0, targetPose.getRotation()));

//         // 3. Controller
//         PPHolonomicDriveController controller = new PPHolonomicDriveController(Constants.PathPlannerConstants.PP_TRANSLATION_PID, Constants.PathPlannerConstants.PP_ROTATION_PID);

//         // 4. Follow command
        
//         followCommand = new FollowPathCommand(
//             path,
//             drivetrain::getPose,
//             drivetrain::getCurrentRobotRelativeSpeeds,
//             (speeds, feedforwards) -> drivetrain.setControl(
//                 new SwerveRequest.RobotCentric()
//                     .withVelocityX(speeds.vxMetersPerSecond)
//                     .withVelocityY(speeds.vyMetersPerSecond)
//                     .withRotationalRate(speeds.omegaRadiansPerSecond)
//             ),
//             new PPHolonomicDriveController(
//                 Constants.PathPlannerConstants.PP_TRANSLATION_PID,
//                 Constants.PathPlannerConstants.PP_ROTATION_PID),
//             RobotConfig.fromGUISettings(),
//             () -> DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red)
//             .orElse(false),
//             drivetrain
//             );

//         followCommand.schedule();
//     }

//     @Override
//     public boolean isFinished() {
//         return followCommand == null || followCommand.isFinished();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (followCommand != null) {
//             followCommand.cancel();
//         }
//         drivetrain.applyRequest(() -> new SwerveRequest.Idle());
//     }
// }