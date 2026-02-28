// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeExtenderConstants{
    public static final int leaderExtenderID = 2;
    public static final String leaderExtenderCANBus = "canivore";
    public static final int followerExtenderID = 3;
    public static final String followerExtenderCANBus = "canivore";

    public static final int IntakeGearRatio = 12;

    public static final int LEFT_ENCODER_DIO = 0;   // DIO port for left encoder (left = leader)
    public static final int RIGHT_ENCODER_DIO = 1;  // DIO port for right encoder

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;

    public static final double MOTION_CRUISE_VELOCITY = 600;
    public static final double MOTION_ACCELERATION = 1200;

    public static final double UP_POSITION_DEGREES = 0;
    public static final double DOWN_POSITION_DEGREES = 90;

    public static final double POSITION_TOLERANCE_DEGREES = 2.0;
    public static final double MAX_CURRENT_AMPS = 40.0;

    public static final boolean INVERT_LEADER_MOTOR = false;
    public static final boolean INVERT_FOLLOWER_MOTOR = true;

    public static final int ENCODER_TICKS_PER_REVOLUTION = 2048;
    public static final double SLEW_RATE_LIMITER = 3.0; // units per second
  }
  
public static class IndexConstants {
    public static final int horizontalIndexMotorID = 4;
    public static final String horizontalIndexMotorCANBus = "canivore";

    public static final int verticalIndexMotorID = 5;
    public static final String verticalIndexMotorCANBus = "canivore";

    public static final int verticalIndexFollowerID = 6;
    public static final String verticalIndexFollowerCANBus = "canivore";

    public static final boolean INVERT_FOLLOWER = false;

    public static final double HORIZONTAL_INDEX_SPEED = 0.3;
    public static final double VERTICAL_INDEX_SPEED = 0.3;

    public static final double INDEX_CURRENT_LIMIT = 30.0; // amps
    public static final boolean INDEX_CURRENT_LIMIT_ENABLE = true;
   }

   public static class IntakeConstants {
    public static final int IntakeMotorID = 199;
    public static final String IntakeMotorCANBus = "canivore";
   }

  //Arrow
  public static class arrowConstants{
    public static final double kPstart = 15;
    public static final double kDstart = 0.2;
    public static final double kIstart = 0.0;
    public static final double errorBand = 0.01;
    public static final double kVstart = 0.0;

    public static final double arrrowManualSpeed = 7;
  }

//Pigeon
  public static class pigeonConstants{
    public static final int Pigeon2 = 1;
  }      

//Speed/drive
  public static class movementConstants{
public static final double maxSpeed = 1.0;
public static final double autonIdleTime = 5.0;
  }
}
