package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakeCommand extends Command {
    IntakeSubsystem _index;
    public ToggleIntakeCommand(IntakeSubsystem index) {
        super();
        _index = index;
        addRequirements(index);
    }
    boolean IsDone = false;

    @Override
    public void initialize() {
        IsDone= false;
    }

    @Override
    public void execute() {
        _index.ToggleIntake();
        IsDone = true;
    }
    @Override
    public boolean isFinished() {
        
        return IsDone;
    }

    
    
}
