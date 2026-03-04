// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CandleConstants;
import frc.robot.subsystems.CandleSubsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CANdleSetColorCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final CandleSubsystem m_subsystem;
  private final Color m_color;
  private final double m_strobe;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CANdleSetColorCommand(CandleSubsystem subsystem, Color colorTarget, double strobeFrequency) {
    m_subsystem = subsystem;
    m_color = colorTarget;
    m_strobe = strobeFrequency;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_strobe > 0)
    {
      m_subsystem.setStrobe(m_color, m_strobe);
    }
    else
    {
      m_subsystem.setColor(m_color);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
