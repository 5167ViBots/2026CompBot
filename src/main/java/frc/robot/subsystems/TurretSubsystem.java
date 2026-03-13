// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TurretSubsystem extends SubsystemBase {
  
  public TalonFX turretMotor;

  private final MotionMagicVoltage turretMotionMagicRequest = new MotionMagicVoltage(0);

  private double currentTargetDegrees = 0;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {

    turretMotor = new TalonFX(Constants.TurretConstants.turretMotorID, Constants.TurretConstants.turretMotorCANBus);
    turretMotor.getPosition().setUpdateFrequency(100);

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.TURRET_CURRENT_LIMIT;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

// changed peak forward & reverse duty cycles from 0.3 to 1.0 since we added a 10:1 gear ratio
    turretConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    turretConfig.MotorOutput.PeakReverseDutyCycle = -1.0;

    turretConfig.MotorOutput.Inverted = Constants.TurretConstants.INVERT_MOTOR ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.FORWARD_SOFT_LIMIT;
    
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.REVERSE_SOFT_LIMIT;

    // PID Control (motion magic)

    turretConfig.Slot0 = new Slot0Configs()
        .withKP(Constants.TurretConstants.kP)
        .withKI(Constants.TurretConstants.kI)
        .withKD(Constants.TurretConstants.kD);
      
    // turretConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.MOTION_CRUISE_VELOCITY / 360 * Constants.TurretConstants.GEAR_RATIO; // convert from degrees/s to ticks/s
    // turretConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.MOTION_ACCELERATION / 360 * Constants.TurretConstants.GEAR_RATIO; // convert from degrees/s^2 to ticks/s^2

    turretMotor.getConfigurator().apply(turretConfig);

  }

  public void setPositionDegrees(double positionDegrees) {

    // Normalize to +-180 (rather than 0-360) [this should be done in ballisticsCalculator, but we'll do it again to make SURE]
    positionDegrees = (((positionDegrees + 180) % 360) - 180);

    // If we're beyond 90 degrees, go to nearest 90
    positionDegrees = Math.min(Math.max(positionDegrees, -90), 90);

    // Convert to motor rotations, then pass the rotations value to MM
    double positionRotaton = degreesToMotorRevolutions(positionDegrees);

    System.out.println("setting Turret to " + positionDegrees + " rotations " + positionRotaton);
    turretMotor.setControl(new PositionDutyCycle(positionRotaton));
    // turretMotionMagicRequest.Position = positionRotaton;
    // turretMotor.setControl(turretMotionMagicRequest);

    // save "current target" for "getCurrentTarget()"
      currentTargetDegrees = positionDegrees;
  }

  public double degreesToMotorRevolutions(double degrees) {

    return ((degrees / 90) * Constants.TurretConstants.FORWARD_SOFT_LIMIT);

    // double revolutionsPerDegree = (Constants.TurretConstants.GEAR_RATIO) / 360.0;
    // return degrees * revolutionsPerDegree;
  }

  public double revolutionsToDegrees(double revolutions) {
    return (revolutions / Constants.TurretConstants.FORWARD_SOFT_LIMIT) * 90;
  }

  public boolean atTargetPosition() {
    double currentPositionRotation = turretMotor.getPosition().getValueAsDouble();
    double targetPositionRotation = degreesToMotorRevolutions(currentPositionRotation);
    double toleranceRotation = degreesToMotorRevolutions(Constants.TurretConstants.POSITION_TOLERANCE_DEGREES);
    if (Math.abs(currentPositionRotation - targetPositionRotation) <= toleranceRotation)
    {
      System.out.println("Turret At Position");
      return true;
    } else return false;
    // return Math.abs(currentPositionRotation - targetPositionRotation) <= toleranceRotation;
  }

  public double getPositionDegrees() {
    // Convert encoder rotations to degrees

    return revolutionsToDegrees(turretMotor.getPosition().getValueAsDouble());
  }

  public double getTargetPositionDegrees() {
    return currentTargetDegrees;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) {
    Slot0Configs slot0 = new Slot0Configs().withKP(kp).withKI(ki).withKD(kd).withKV(kv).withKG(kg);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0 = slot0;
    config.MotionMagic.MotionMagicCruiseVelocity = mmv / 360 * Constants.TurretConstants.GEAR_RATIO;
    config.MotionMagic.MotionMagicAcceleration = mma / 360 * Constants.TurretConstants.GEAR_RATIO;
    turretMotor.getConfigurator().apply(config);
  }

  public void zeroEncoder() {
    turretMotor.setPosition(TurretConstants.HOME_ANGLE_DEGREES);
  }

  public void setPower(double power){
    turretMotor.setControl(new DutyCycleOut(power));
  }

  public void stop() {
    turretMotor.set(0.0);
  }
}
