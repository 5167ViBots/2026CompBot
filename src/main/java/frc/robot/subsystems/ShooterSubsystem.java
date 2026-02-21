// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.revrobotics.Rev2mDistanceSensor;

//Caleb
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HelperClasses.Constants.ShooterSubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX shooter1, shooter2;
 // private Rev2mDistanceSensor distanceSensor;
  /** Creates a new ExampletuneSubsystem. */
  public ShooterSubsystem() {
    shooter1 = new TalonFX(ShooterSubsystemConstants.ShooterMotor1ID, ShooterSubsystemConstants.ShooterMotor1CAN);
    shooter2 = new TalonFX(ShooterSubsystemConstants.ShooterMotor2ID, ShooterSubsystemConstants.ShooterMotor2CAN);
    shooter1.setNeutralMode(NeutralModeValue.Brake);
    shooter2.setNeutralMode(NeutralModeValue.Brake);
    shooter2.setControl(new Follower(shooter1.getDeviceID(), MotorAlignmentValue.Opposed));
   // distanceSensor = null;// new Rev2mDistanceSensor(ShooterSubsystemConstants.distanceSensorPort);

   var slot0Configs = new Slot0Configs();
slot0Configs.kP = ShooterSubsystemConstants.kP; // An error of 1 rotation results in 2.4 V output
slot0Configs.kI = ShooterSubsystemConstants.kI; // no output for integrated error
slot0Configs.kD = ShooterSubsystemConstants.kD; // A velocity of 1 rps results in 0.1 V output

shooter1.getConfigurator().apply(slot0Configs);
  }
public void setspeed(double speed) {
  // create a velocity closed-loop request, voltage output, slot 0 configs
final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

// set velocity to 8 rps, add 0.5 V to overcome gravity
shooter1.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
   

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

  public void shootForward(){

    setspeed(ShooterSubsystemConstants.shooterspeed);
    }

  public void shootBack(){
    setspeed(-ShooterSubsystemConstants.shooterspeed);

  }

  public void warmUp(){
    shooter1.set(1);
    shooter2.set(1);
  }

  public void warmDown(){

  }

  public void shootStop(){

  setspeed(0);
  }

  public void getPosition(){
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