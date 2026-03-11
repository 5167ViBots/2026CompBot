package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class ToggleHorizontalIndexCommand extends Command {
    IndexSubsystem _index;
    public ToggleHorizontalIndexCommand(IndexSubsystem index) {
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
        _index.ToggleHorizontalIndex();
        IsDone = true;
    }
    @Override
    public boolean isFinished() {
        
        return IsDone;
    }

    
    
}
