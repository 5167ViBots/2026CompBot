// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.ShuffleboardControl;
import frc.robot.ShuffleboardControl.MotorAccessor;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
private TalonFX shooterMotor1;
private TalonFX shooterMotor2;

public ShooterSubsystem() {

    shooterMotor1 = new TalonFX(Constants.ShooterConstants.shooterMotor1ID, Constants.ShooterConstants.shootermotor1CANBus);
    // shooterMotor2 = new TalonFX(Constants.ShooterConstants.shootermotor2ID, Constants.ShooterConstants.shootermotor2CANBus);
    var slot0Configs = new Slot0Configs();
    // slot0Configs.kS = Constants.ShooterConstants.kS;
    // slot0Configs.kV = Constants.ShooterConstants.kV;
    slot0Configs.kP = Constants.ShooterConstants.kP;
    slot0Configs.kI = Constants.ShooterConstants.kI;
    slot0Configs.kD = Constants.ShooterConstants.kD;

    shooterMotor1.getConfigurator().apply(slot0Configs);

    // shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(),
    //   Constants.ShooterConstants.INVERT_FOLLOWER
    //     ? MotorAlignmentValue.Opposed
    //     : MotorAlignmentValue.Aligned));
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

  public void shoot(){

    setSpeedMPS(10); // sets speed in m/s.

    // shooterMotor1.setControl(new DutyCycleOut(.525));
    // setSpeed(Constants.ShooterConstants.SHOOTER_SPEED);

  }

  // public double getSpeed(){
  //   return shooterMotor1.getVelocity().getValueAsDouble();
  // }

  public void setSpeedMPS(double speedMPS)
  {
    double speed = MPStoRPS(speedMPS);
    shooterMotor1.setControl(new VelocityDutyCycle(speed));
    // final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    // shooterMotor1.setControl(request.withVelocity(speed).withFeedForward(speed));
  }

  public double MPStoRPS(double speedMPS)
  {
    return speedMPS / 0.319; // wheel has a diameter of 0.319 m ((m/s)/m = 1/s)
  }

  public double RPStoMPS(double speedRPS)
  {
    return speedRPS * 0.319; // wheel has a diameter of 0.319 m
  }

  public void stop(){
    shooterMotor1.setControl(new DutyCycleOut(0));
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
    // shoot();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) {
      Slot0Configs slot0 = new Slot0Configs().withKP(kp).withKI(ki).withKD(kd);
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0 = slot0;
      shooterMotor1.getConfigurator().apply(config);
  }
}
