package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BallisticCalculator;
import frc.robot.Constants;
import frc.robot.FieldConstants;           // for hub positions
import frc.robot.RobotContainer;          // to get current field region / pose
import frc.robot.FieldConstants.FieldRegion;

public class DynamicAimCommand extends Command {

    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final RobotContainer robotContainer;   // for pose, speeds, and field state

    public DynamicAimCommand(TurretSubsystem turret, HoodSubsystem hood, RobotContainer robotContainer) {
        this.turret = turret;
        this.hood = hood;
        this.robotContainer = robotContainer;

        addRequirements(turret, hood);   // important: prevents conflicts
    }

    @Override
    public void execute() {
        Pose2d robotPose = robotContainer.getPose();
        ChassisSpeeds robotSpeeds = robotContainer.getRobotSpeeds();

        // Debug message
        System.out.println("Pose X = "+ robotPose.getX() + " Y = " + robotPose.getY() + " T = " + robotPose.getRotation());

        // Get the correct hub target based on current field region / alliance
        Translation2d targetPos = getTargetPosition();

        double target_height = ((targetPos == FieldConstants.RED_HUB_TARGET) || (targetPos == FieldConstants.BLUE_HUB_TARGET) ? Constants.ballisticConstants.HUB_HEIGHT : 0.0);

        BallisticCalculator.BallisticSolution solution = 
            BallisticCalculator.getAngles(
                robotPose,
                targetPos,
                robotSpeeds,
                target_height
            );

        turret.setPositionDegrees(solution.turretAngleDegrees);
        hood.setPositionDegrees(solution.hoodAngleDegrees);
    }

    private Translation2d getTargetPosition() {
        // Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        FieldConstants.FieldRegion region = robotContainer.getFieldLocation();


        // removed alliance-specific targetting

        // boolean isBlue = alliance == Alliance.Blue;

        // if (isBlue) {
            if ((region == FieldRegion.BLUE_DEEP_LEFT)
                || (region == FieldRegion.BLUE_FRONT_LEFT)
                || (region == FieldRegion.BLUE_DEEP_RIGHT)
                || (region == FieldRegion.BLUE_FRONT_RIGHT))
                return FieldConstants.BLUE_HUB_TARGET;
            else if ((region == FieldRegion.NEUTRAL_LEFT)
                || (region == FieldRegion.BLUE_LEFT_TRENCH)
                || (region == FieldRegion.BLUE_LEFT_BUMP))
                return FieldConstants.BLUE_LEFT_TARGET;
            else return FieldConstants.BLUE_RIGHT_TARGET;

        // }
        // else{
        //     if ((region == FieldRegion.RED_DEEP_LEFT)
        //         || (region == FieldRegion.RED_FRONT_LEFT)
        //         || (region == FieldRegion.RED_DEEP_RIGHT)
        //         || (region == FieldRegion.RED_FRONT_RIGHT)
        //         || (region == FieldRegion.RED_LEFT_BUMP)
        //         || (region == FieldRegion.RED_LEFT_TRENCH))
        //             return FieldConstants.RED_HUB_TARGET;
        //     else if (region == FieldRegion.NEUTRAL_LEFT) return FieldConstants.RED_LEFT_TARGET;
        //     else return FieldConstants.RED_RIGHT_TARGET;
        // }
    }

    @Override
    public boolean isFinished() {
        return false;   // runs until interrupted
    }
}