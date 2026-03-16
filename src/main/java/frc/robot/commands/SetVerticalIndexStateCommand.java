package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class SetVerticalIndexStateCommand extends Command {
    IndexSubsystem _index;
    boolean _DesiredState;
    public SetVerticalIndexStateCommand(IndexSubsystem index, Boolean DesiredState) {
        super();
        _index = index;
        _DesiredState = DesiredState;
        addRequirements(index);
    }
    boolean IsDone = false;

    @Override
    public void initialize() {
        IsDone= false;
    }


    @Override
    public void execute() {
        _index.SetVerticalIndexState(_DesiredState);
        IsDone = true;
    }
    @Override
    public boolean isFinished() {
        
        return IsDone;
    }

    
    
}
