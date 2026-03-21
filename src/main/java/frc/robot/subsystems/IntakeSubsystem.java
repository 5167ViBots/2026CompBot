// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ShuffleboardControl;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonFX intakeMotor1;

  public IntakeSubsystem() {

    intakeMotor1 = new TalonFX(Constants.IntakeConstants.intakeMotor1ID, Constants.IntakeConstants.intakemotor1CANBus);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intakeSubsystemCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
 boolean armed = false;
 
  public void setPower(double powerPercent){
    if (powerPercent > 0)
      armed = true;

    // if (powerPercent == 0 && armed){
    //   int x = 1/ (1-1);
    // }
    intakeMotor1.setControl(new DutyCycleOut(powerPercent));
    // System.out.println("Setting power: " + powerPercent);
  }

  public double getPower(){
    return intakeMotor1.getDutyCycle().getValueAsDouble();
  }
  
  public void intake(){
    setPower(Constants.IntakeConstants.INTAKE_POWER_PERCENT);
  }

  public void stop(){
    setPower(0.0);
  }

  public void reverseIntake(){
    setPower(-Constants.IntakeConstants.INTAKE_POWER_PERCENT);
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

   boolean RunIntake = false;

    public void ToggleIntake() {
        
        RunIntake = !RunIntake;
    }

    
    public void SetIntakeState(Boolean State) {
        
        RunIntake = State;
    }


  @Override
  public void periodic() {

    if (RunIntake){
      intake();
    }else{
      stop();
    }
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
