// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.arrowConstants;
import frc.robot.Constants.pigeonConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ser.impl.AttributePropertyWriter;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import java.lang.reflect.GenericArrayType;

import javax.net.ssl.KeyStoreBuilderParameters;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ArrowAutoSubsystem extends SubsystemBase {

  private final TalonFX arrow;
  private final Pigeon2 pigeon;

  double newPosition;
  public double change;

  



  private final ShuffleboardTab tab = Shuffleboard.getTab("PID Tuner");

  private final GenericEntry kpEntry = tab.add("kP",arrowConstants.kPstart)
    .withPosition(0, 0).getEntry();
  private final GenericEntry kiEntry = tab.add("ki",arrowConstants.kIstart)
    .withPosition(1, 0).getEntry();
  private final GenericEntry kdEntry = tab.add("kD",arrowConstants.kDstart)
    .withPosition(2, 0).getEntry();
  private final GenericEntry kvEntry = tab.add("kV",0.0)
    .withPosition(3, 0).getEntry();

  private final GenericEntry errorEntry = tab.add("error",0.0)
    .withWidget(BuiltInWidgets.kGraph)
    .withPosition(4,0).getEntry();

  private final GenericEntry applyButton = tab.add("apply",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(1,1)
    .getEntry();

    private final GenericEntry activeControl = tab.add("active",false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .withPosition(2,1)
    .getEntry();

    private final GenericEntry motorPower = tab.add("motor power", 0.0)
    .withPosition(1,3)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  public ArrowAutoSubsystem() {
    //define arrow

    pigeon = new Pigeon2(pigeonConstants.Pigeon2, "canivore");
    arrow = new TalonFX(8,"canivore");

    TalonFXConfiguration arrowConfig = new TalonFXConfiguration();
    arrowConfig.Slot0 = new Slot0Configs()
    .withKP(arrowConstants.kPstart)
    .withKI(arrowConstants.kIstart)
    .withKD(arrowConstants.kDstart);

  arrow.getConfigurator().apply(arrowConfig);

  }

public void setPosition(double positionDegrees) {

  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  final double deg2rot = positionDegrees/360;
  final double currentPosition = arrow.getPosition().getValueAsDouble();
  final double error = deg2rot - currentPosition;
  errorEntry.setDouble(error);
  motorPower.setDouble(arrow.getMotorVoltage().getValueAsDouble());

  if(Math.abs(error) > arrowConstants.errorBand){
    arrow.setControl(m_request.withPosition(deg2rot));
    activeControl.setBoolean(true);
  }
  else activeControl.setBoolean(false);

}


private void applyGainsFromShuffleboard(){
  if (applyButton.getBoolean(false) == true){
    TalonFXConfiguration shuffleboardConfig = new TalonFXConfiguration();
    shuffleboardConfig.Slot0 = new Slot0Configs()
    .withKP(kpEntry.getDouble(arrowConstants.kPstart))
    .withKI(kiEntry.getDouble(arrowConstants.kIstart))
    .withKD(kdEntry.getDouble(arrowConstants.kDstart))
    .withKV(kvEntry.getDouble(arrowConstants.kVstart));

    arrow.getConfigurator().apply(shuffleboardConfig);
    arrow.getConfigurator().refresh(shuffleboardConfig);

    applyButton.setBoolean(false);
    System.out.println("Applying configs KP = " +
  
    shuffleboardConfig.Slot0.kP);
  }
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
          /* one-time action goes hecurrentPositionre */
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
        // System.out.println(pigeon.getYaw());
        applyGainsFromShuffleboard();
        setPosition(-(pigeon.getYaw().getValueAsDouble() + newPosition));
        
    }
    
  public void changeAngle(double change) {
    newPosition = newPosition + change;

  }
    
      @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
