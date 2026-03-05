// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean ENABLE_LIMELIGHT_INITIALIZATION = true;
  public static final boolean ENABLE_TURRET_HOMING = false;

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

    public static final int kButtonBoardPort = 1;

    public static final int FIRE_ON_THE_MOVE_BUTTON = 1;
    public static final int PUSH_BUTTON = 2;
    public static final int CLIMB_BUTTON = 3;
    public static final int IDLE_BUTTON = 4;
    public static final int UNJAM_BUTTON = 5;
    public static final int STOP_AND_SHOOT_BUTTON = 6;

    public static final int FIRE_BUTTON = 7;

    public static final int CLIMB_RAISE_ARM_BUTTON = 8;
    public static final int CLIMB_LOWER_ARM_BUTTON = 9;
    public static final int TOGGLE_DRIVE_MODE_BUTTON = 10;
  }

  public static class IntakeConstants {
    public static final int intakeMotor1ID = 45;
    public static final String intakemotor1CANBus = "rio";
    public static final double INTAKE_POWER_PERCENT = 0.4;
  }

  public static class ShooterConstants {
    public static final int shooterMotor1ID = 0;
    public static final String shootermotor1CANBus = "rio";

    public static final int shootermotor2ID = -1;
    public static final String shootermotor2CANBus = "rio";

    public static final double SHOOTER_SPEED = 10.0;

    public static final double kS = 0.2; // Static gain (volts)
    public static final double kP = 0.1; // Proportional gain (volts per unit error)
    public static final double kV = 0.1; // Velocity gain (volts per unit velocity)
    public static final double kI = 0.0; // Integral gain (volts per unit integral)
    public static final double kD = 0.0; // Derivative gain (volts per unit derivative)

    public static final double FEED_FORWARD = 0.05; // Additional feedforward voltage to help overcome static friction

    public static final boolean INVERT_FOLLOWER = true;
  }

  public static class IntakeExtenderConstants{
    public static final int leaderExtenderID = 46;
    public static final String leaderExtenderCANBus = "rio";
    public static final int followerExtenderID = 48;
    public static final String followerExtenderCANBus = "rio";

    public static final int IntakeGearRatio = 12;

    public static final int LEFT_ENCODER_DIO = 2;   // DIO port for left encoder (left = leader)
    public static final int RIGHT_ENCODER_DIO = 1;  // DIO port for right encoder

    public static final double kP = 5.0;
    public static final double kI = 0.015;
    public static final double kD = 0.25;
    public static final double kV = 0.0;
    public static final double kG = 0.0;

    public static final double MOTION_CRUISE_VELOCITY = 600;
    public static final double MOTION_ACCELERATION = 1200;

    public static final double UP_POSITION = 0.33;
    public static final double DOWN_POSITION = 0.0;

    public static final double POSITION_TOLERANCE_DEGREES = 2.0;
    public static final double MAX_CURRENT_AMPS = 40.0;

    public static final boolean INVERT_LEADER_MOTOR = true;
    public static final boolean INVERT_FOLLOWER_MOTOR = false;

    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
    public static final double SLEW_RATE_LIMITER = 3.0; // units per second
  }

  public static class IndexConstants {
    public static final int horizontalIndexMotorID = 17;
    public static final String horizontalIndexMotorCANBus = "rio";

    public static final int verticalIndexMotorID = 18;
    public static final String verticalIndexMotorCANBus = "rio";

    public static final int verticalIndexFollowerID = -1;
    public static final String verticalIndexFollowerCANBus = "rio";

    public static final boolean INVERT_FOLLOWER = false;

    public static final double HORIZONTAL_INDEX_SPEED = 0.5;
    public static final double VERTICAL_INDEX_SPEED = 0.5;

    public static final double INDEX_CURRENT_LIMIT = 30.0; // amps
    public static final boolean INDEX_CURRENT_LIMIT_ENABLE = true;
   }

   public static class TurretConstants {
      public static final int turretMotorID = 77;
      public static final String turretMotorCANBus = "rio";

      public static final double kP = 0.8;
      public static final double kI = 0.0; 
      public static final double kD = 12.0;

      public static final double MOTION_CRUISE_VELOCITY = 200;
      public static final double MOTION_ACCELERATION = 800;

      public static final double ENCODER_TICKS_PER_REVOLUTION = 2048;
      public static final double SLEW_RATE_LIMITER = 3.0;           // units per second

      public static final double MOTOR_TOOTH_COUNT = 22.0;
      public static final double TURRET_TOOTH_COUNT = 120.0;

      public static final double GEAR_RATIO = TURRET_TOOTH_COUNT / MOTOR_TOOTH_COUNT;

      public static final double TURRET_CURRENT_LIMIT = 30.0;       // amps

      public static final double POSITION_TOLERANCE_DEGREES = 2;    // doesn't need to be perfectly accurate, we have a +-3' window

      public static final double FORWARD_SOFT_LIMIT_DEGREES =  180; // forward limit of turret rotation in degrees
      public static final double REVERSE_SOFT_LIMIT_DEGREES = -180; // reverse limit of turret rotation in degrees

      public static final boolean INVERT_MOTOR = false;

      public static final double LARGE_ERROR = 15.0;                // define "way off" for "disable turret while it is way off target"

      public static final int HOMING_SWITCH_PORT = 15;       // port for the sensor
      public static final double HOMING_SPEED = 0.15;       // slow and safe
      public static final double  HOMING_TIMEOUT = 8.0;     // seconds
      public static final double HOME_ANGLE_DEGREES = 0.0;  // what "home" means (we may need to adjust this.)
   }

   public static class HoodConstants {
      public static final int hoodMotorID = 31;
      public static final String hoodMotorCANBus = "rio";

      public static final double kP = 0.8;
      public static final double kI = 0.0;
      public static final double kD = 12.0;
      public static final double kV = 0.0;
      public static final double kG = 0.04; // gravity feedforward gain, needs to be tuned based on weight of hood and friction in system

      public static final double MOTION_CRUISE_VELOCITY = 80;
      public static final double MOTION_ACCELERATION = 400;

      public static final double UPPER_LIMIT = 90;
      public static final double LOWER_LIMIT = 20;

      public static final double GEAR_RATIO = 25.0; // gearing ratio on motor

      public static final double POSITION_TOLERANCE_DEGREES = 1.0;
      public static final double MAX_CURRENT_AMPS = 40.0;

      public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
      public static final double SLEW_RATE_LIMITER = 3.0; // units per second

      public static final double FORWARD_SOFT_LIMIT_DEGREES = LOWER_LIMIT; // forward limit of hood rotation in degrees
      public static final double REVERSE_SOFT_LIMIT_DEGREES = UPPER_LIMIT; // reverse limit of hood rotation in degrees

      public static final double HOOD_CURRENT_LIMIT = 30.0; // amps

      public static final boolean INVERT_MOTOR = false;

      public static final double FF_CAP = 0.3; // cap for feedforward to prevent excessive values
   }

   public static class robotStates {
      public enum State{
        FIRE_ON_THE_MOVE, // intake, index, and shoot on the move, disable climb motors
        STOP_AND_SHOOT,   // stop the robot and shoot, no fire on the move, disable climb motors
        PUSH,             // motor problems, switch to defensive mode
        CLIMB,            // active during endgame - disengage other motors and engage climb motors
        IDLE,             // default state, all motors off
        CALIBRATION       // for initial calibration of motors
      }
  }

  public static class ballisticConstants{
    public static final double GRAVITY = 9.81; // m/s^2
    public static final double SHOOTER_EXIT_VELOCITY = Units.feetToMeters(30);  // 30 ft/s converted to m/s, needs to be tuned based on shooter performance
    public static final double HUB_HEIGHT = Units.feetToMeters(6);              // height of hub center from floor
    public static final double SHOOTER_HEIGHT = Units.feetToMeters(2);          // height of shooter exit point from floor, needs to be measured

    public static final int MAX_ATTEMPTS = 10; // max iterations for binary search in ballistic calculator

    public static final double HOOD_HEIGHT_ERROR_TOLERANCE = Units.feetToMeters(1); // height of tolerance for hood height solution in ballistic calculator
  }

  public static class BoundaryConstants{
    public static final double ROBOT_MARGIN = 0.35;
    public static final double SLOWDOWN_MARGIN = 1.5;
    public static final double HARD_STOP_MARGIN = 0.25;
    public static final double MaxSpeed = 1.0;
  }

  public static class AutoConstants{
    public static final int TRANSLATION_PID = 1;
    public static final int ROTATION_PID = 2;
  }

  public static class CandleConstants{
    public static final int CANDLE_DEVICE_ID = 99;
    public static final String CANDLE_CANBUS = "canivore";

    // robot state colors
    public static final Color READY_TO_SHOOT_COLOR  = Color.kGreen;             // default state is the robot ready to shoot
    public static final Color RED_ALLIANCE_COLOR    = Color.kRed;               // fallthrough colors are alliance
    public static final Color BLUE_ALLIANCE_COLOR   = Color.kBlue;              // alliance blue 
    public static final Color UNJAM_COLOR           = Color.kYellow;            // Color to set CANdle when unjam button is pressed
    public static final Color IDLE_COLOR            = Color.kWhite;             // Color to set CANdle when robot is in IDLE state
    public static final Color ERROR_COLOR           = Color.kOrange;            // Error color (this is bad)
    public static final Color TURRET_AIMING         = Color.kPurple;            // Turret aiming (and not ready to shoot)
    public static final Color CLIMB_COLOR           = Color.kDarkOliveGreen;    // Switch to green when Climb state
    public static final Color PUSH_COLOR            = Color.kWhite;             // will also strobe

    public static final Color DEFAULT_COLOR         = Color.kWhite ;            // setting default.

    public static final double NO_STROBE = 0.0;
    public static final double DEFAULT_STROBE_HZ = 4.0;                         // 4 cycles/second is default strobe
    public static final double FAST_STROBE_HZ = 8.0;                            // 8 cycles/second is fast strobe
  }

  public static class ClimbConstants{
    public static final int ARM_MOTOR_ID = 7;
    public static final String ARM_MOTOR_CANBUS = "rio";
    public static final int ARM_GEAR_RATIO = 90;

    public static final double ARM_POWER = 0.4;

    public static final double ARM_MIN = 0;
    public static final double ARM_MAX = 100;
  }

  public static class PathPlannerConstants {
    public static final PIDConstants PP_TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants PP_ROTATION_PID = new PIDConstants(3.0, 0.0, 0.0);
  }
}