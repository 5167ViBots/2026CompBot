// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX armMotor;

  // private final MotionMagicVoltage armMotionMagic = new MotionMagicVoltage(0);
  private final MotionMagicVoltage hookMotionMagic = new MotionMagicVoltage(0);

  public ClimbSubsystem() {
    
    armMotor = new TalonFX(Constants.ClimbConstants.ARM_MOTOR_ID, Constants.ClimbConstants.ARM_MOTOR_CANBUS);
    
    TalonFXConfiguration armcfg = new TalonFXConfiguration();

    armcfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armcfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armcfg.Feedback.SensorToMechanismRatio = Constants.ClimbConstants.ARM_GEAR_RATIO;
    armcfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Arm is not velocity or position controlled, so we don't need motion magic or PID.

    // armcfg.MotionMagic.MotionMagicCruiseVelocity = Constants.ClimbConstants.ARM_MOTION_CRUISE_VELOCITY;
    // armcfg.MotionMagic.MotionMagicAcceleration = Constants.ClimbConstants.ARM_MOTION_ACCELERATION;
    // armcfg.MotionMagic.MotionMagicJerk = Constants.ClimbConstants.ARM_MOTION_JERK;

    // armcfg.Slot0.kP = Constants.ClimbConstants.ARM_kP;
    // armcfg.Slot0.kI = Constants.ClimbConstants.ARM_kI;
    // armcfg.Slot0.kD = Constants.ClimbConstants.ARM_kD;
    // armcfg.Slot0.kV = Constants.ClimbConstants.ARM_kV;
    // armcfg.Slot0.kA = Constants.ClimbConstants.ARM_kA;

    armMotor.getConfigurator().apply(armcfg);

    // armMotionMagic.Position = 0;
  }

  public void stopArm() {
    setArmPower(0);
  }

  public void setArmPower(double armPower) {
    // if (((armPower > 0) && (getArmPosition() < Constants.ClimbConstants.ARM_MAX)) || ((armPower < 0) && (getArmPosition() > Constants.ClimbConstants.ARM_MIN))){
      armMotor.setControl(new DutyCycleOut(armPower));
    // }
    // if (armPower == 0) armMotor.setControl(new DutyCycleOut(0));
  }

  public double getPower(){
    return armMotor.getDutyCycle().getValueAsDouble();
  }

  public void raiseArm(){
    setArmPower(Constants.ClimbConstants.ARM_POWER);
    System.out.println("Climbing up");
  }

  public void lowerArm(){
    setArmPower(-Constants.ClimbConstants.ARM_POWER);

    System.out.println("Climbing down");  }

  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }
}