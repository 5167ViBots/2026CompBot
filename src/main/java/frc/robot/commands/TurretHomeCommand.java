package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants.TurretConstants;

public class TurretHomeCommand extends Command {

    private final TurretSubsystem turret;
    private final DigitalInput homeSwitch;

    private double startTime;
    private boolean homed = false;

    public TurretHomeCommand(TurretSubsystem turret) {
        this.turret = turret;
        this.homeSwitch = new DigitalInput(TurretConstants.HOMING_SWITCH_PORT);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        homed = false;
        startTime = Timer.getFPGATimestamp();

        // Rotate slowly in one direction (choose direction that hits switch safely)
        turret.setPower(-TurretConstants.HOMING_SPEED);   // negative = one direction
    }

    @Override
    public void execute() {
        // Most limit switches read false when pressed
        if (!homeSwitch.get()) {
            turret.stop();
            turret.zeroEncoder();        // set encoder to 0°
            homed = true;
        }
    }

    @Override
    public boolean isFinished() {
        return homed || (Timer.getFPGATimestamp() - startTime > TurretConstants.HOMING_TIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        if (homed) {
            System.out.println("Turret homing SUCCESS - zeroed at 0°");
        } else {
            System.out.println("Turret homing TIMEOUT - check switch!");
        }
    }
}