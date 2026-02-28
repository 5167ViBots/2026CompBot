// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

  private TalonFX horizontalIndexMotor;
  private TalonFX verticalIndexMotor;

  /** Creates a new IndexSubsystem. */
  public IndexerSubsystem() {

    horizontalIndexMotor = new TalonFX(Constants.IndexConstants.horizontalIndexMotorID, Constants.IndexConstants.horizontalIndexMotorCANBus);
    verticalIndexMotor = new TalonFX(Constants.IndexConstants.verticalIndexMotorID, Constants.IndexConstants.verticalIndexMotorCANBus);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = Constants.IndexConstants.INDEX_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexConstants.INDEX_CURRENT_LIMIT_ENABLE;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    horizontalIndexMotor.getConfigurator().apply(config);
    verticalIndexMotor.getConfigurator().apply(config);
  }

  public void horizontalSpeed(double velocity) {
    horizontalIndexMotor.setControl(new DutyCycleOut(velocity));
  }

  public void verticalSpeed(double velocity) {
    verticalIndexMotor.setControl(new DutyCycleOut(velocity));
  }

  public void stop() {
    horizontalSpeed(0.0);
    verticalSpeed(0.0);
  }

  public void runIndex() {
    horizontalSpeed(Constants.IndexConstants.HORIZONTAL_INDEX_SPEED);
    verticalSpeed(Constants.IndexConstants.VERTICAL_INDEX_SPEED);
  }

  public void reverseIndex() {
    horizontalSpeed(-Constants.IndexConstants.HORIZONTAL_INDEX_SPEED);
    verticalSpeed(-Constants.IndexConstants.VERTICAL_INDEX_SPEED);
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
}