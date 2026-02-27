package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Import the imports here

public class DeflectorSubsystem extends SubsystemBase {

//
//Use set position / run to position to set the deflector 
//based on where it needs to be to make sure fuel reaches the hub
//
    
  public DeflectorSubsystem() {}


  
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
