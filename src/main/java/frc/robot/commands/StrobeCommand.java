// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LightSubsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class StrobeCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final LightSubsystem m_subsystem;
  private final Color InColor;
  private final double frequency = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StrobeCommand(LightSubsystem subsystem, Color Incolor, double frequency) {
    m_subsystem = subsystem;
    InColor = Incolor;
    frequency = frequency;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetTimer();
    m_subsystem.startTimer();
    m_subsystem.strobeColor(InColor, frequency);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_subsystem.setColor(Color.kAliceBlue);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
