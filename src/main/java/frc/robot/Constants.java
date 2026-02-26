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
