package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BallisticCalculator;
import frc.robot.Constants;
import frc.robot.FieldConstants;           // for hub positions
import frc.robot.RobotContainer;          // to get current field region / pose
import frc.robot.BallisticCalculator.BallisticSolution;
import frc.robot.FieldConstants.FieldRegion;

public class StaticAimCommand extends Command {

    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final RobotContainer robotContainer;   // for pose, speeds, and field state

    private BallisticSolution solution;

    public StaticAimCommand(TurretSubsystem turret, HoodSubsystem hood, ShooterSubsystem shooter, RobotContainer robotContainer) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;
        this.robotContainer = robotContainer;

        solution = new BallisticSolution(0, 0, 0);

        addRequirements(turret, hood, shooter);   // important: prevents conflicts
    }


    // Run through the ballistics calculator once

    @Override
    public void initialize() {
    }

    // Set positions constantly
    @Override
    public void execute() {
        calculateValues();

        turret.setPositionDegrees(solution.turretAngleDegrees);
        hood.setPositionDegrees(solution.hoodAngleDegrees);

        if (Double.isNaN(solution.shooterSpeedMps))
        {
            shooter.setSpeedMPS(Constants.ShooterConstants.SHOOTER_SPEED);
        }
        else
        {
            shooter.setSpeedMPS(solution.shooterSpeedMps * 2);
        }

    }


    public void calculateValues()
    {
        Pose2d robotPose = robotContainer.getPose();
        ChassisSpeeds robotSpeeds = robotContainer.getRobotSpeeds();

        // Get the correct hub target based on current field region / alliance
        Translation2d targetPos = getTargetPosition();

        double target_height = ((targetPos == FieldConstants.RED_HUB_TARGET) || (targetPos == FieldConstants.BLUE_HUB_TARGET) ? Constants.ballisticConstants.HUB_HEIGHT : 0.0);

        // Debug message
        System.out.println("Pose X = "+ robotPose.getX() + " Y = " + robotPose.getY() + " T = " + robotPose.getRotation());
        System.out.println("Target X = "+ targetPos.getX() + " Y = " + targetPos.getY());
        System.out.println("Target " + (targetPos == FieldConstants.BLUE_HUB_TARGET ? "Hub" : "Field"));

        solution = 
            BallisticCalculator.getAngles(
                robotPose,
                targetPos,
                robotSpeeds,
                target_height
            );

        System.out.println("hood Solution: " + solution.hoodAngleDegrees);
        System.out.println("TurretSolution: " + solution.turretAngleDegrees);
        System.out.println("Shooter Solution: " + solution.shooterSpeedMps);

    }
    private Translation2d getTargetPosition() {
        // Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        FieldConstants.FieldRegion region = robotContainer.getFieldLocation();

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

    }

    @Override
    public boolean isFinished() {
        return false;
        // return turret.atTargetPosition(); // we're only running through this once, turret.setposition() should fix the turret angle
    }
}