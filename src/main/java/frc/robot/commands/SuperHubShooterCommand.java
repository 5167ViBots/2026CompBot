// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.GameState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SuperHubShooterCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ShooterSubsystem m_subsystem;
  // private final IndexSubsystem b_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SuperHubShooterCommand(ShooterSubsystem subsystem){
    m_subsystem = subsystem;
    // b_subsystem = bsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double DistanceFromHub = GameState.GetDistanceFromHub();

        //These are pre tuned values taken from power/rps measurements from successful shots. 
        double MaxDistance = 3.855; 
        double MinDistance = 2;
        double DistancePercent = (DistanceFromHub / 1.855) - 1;

        double DesiredVoltage = (1.4*DistancePercent)+5.6;
        double DesiredRPS = (10*DistancePercent)+45;


        m_subsystem.shootVoltage(DesiredVoltage);
        // b_subsystem.runIndex();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
