// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TurretHomeCommand;

import frc.robot.Constants;
import frc.robot.FieldConstants.FieldRegion;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        // Update field & pose every 20 ms, offset 5 ms.
        addPeriodic(m_robotContainer::updateFieldAndPoseDisplay, 0.020, 0.005);
        
        // Check limelight every 100 ms, offset 10 ms.
        addPeriodic(m_robotContainer.getLimelightSide()::periodic, 0.100, 0.10);
        addPeriodic(m_robotContainer.getLimelightFront()::periodic, 0.100, 0.10);

        // update swerve odometry with Limelight vision every 100 ms, offset 35 ms.
        addPeriodic(m_robotContainer::updatePoseWithVision, 0.100, 0.035);
        addPeriodic(m_robotContainer::updatePoseWithVisionSide, 0.100, 0.055);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        if (Constants.ENABLE_LIMELIGHT_INITIALIZATION) { 
            m_robotContainer.resetPoseFromLimelight();
        }

       m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }


    }

    @Override
    public void autonomousPeriodic() {
        // dynamicAim();
    }

    public void dynamicAim() {
        Pose2d robotPose = m_robotContainer.getPose();
        ChassisSpeeds robotSpeeds = m_robotContainer.getRobotSpeeds();

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

        m_robotContainer.turret.setPositionDegrees(solution.turretAngleDegrees);
        m_robotContainer.hood.setPositionDegrees(solution.hoodAngleDegrees);
    }

    private Translation2d getTargetPosition() {
        // Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        FieldConstants.FieldRegion region = m_robotContainer.getFieldLocation();


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
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
                if (Constants.ENABLE_LIMELIGHT_INITIALIZATION) { 
            m_robotContainer.resetPoseFromLimelight();
        }
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        
        // only initialize from limelight if "Initialization" constant is true (change in Constants.java)
        // We will likely move this to AutonomousInit when we're confident that everything works well.

        if (Constants.ENABLE_TURRET_HOMING){                                // only initialize turret if HOMING is true. (change in Constants.java)
            CommandScheduler.getInstance().schedule(                        // eventually move this to AutonomousInit when we're done testing.
                new TurretHomeCommand(m_robotContainer.getTurret())
            );
        }
    }

    @Override
    public void teleopPeriodic() {
        // ShuffleboardControl.update();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
