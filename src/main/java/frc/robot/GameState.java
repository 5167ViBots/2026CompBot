package frc.robot; 


import frc.robot.subsystems.CommandSwerveDrivetrain;

public class GameState {
    
    final static CommandSwerveDrivetrain drive = RobotContainer.TheRobot.drivetrain;
    
        public static double GetDistanceFromHub()
        {
            return drive.getState().Pose.getTranslation().getDistance(FieldConstants.BLUE_HUB_TARGET);
    }
}