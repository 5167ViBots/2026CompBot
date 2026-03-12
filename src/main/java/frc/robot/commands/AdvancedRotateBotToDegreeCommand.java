package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.annotation.Target;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AdvancedRotateBotToDegreeCommand extends Command {
    CommandSwerveDrivetrain drivetrain;
    double TargetDegree;
    double ErrorRate;

    public AdvancedRotateBotToDegreeCommand(double DegreeToRotateTo, double AcceptableErrorRate,
            CommandSwerveDrivetrain _drivetrain) {
        super();
        drivetrain = _drivetrain;
        addRequirements(_drivetrain);
        TargetDegree = DegreeToRotateTo;
        ErrorRate = AcceptableErrorRate;
    }

    PIDController pid;

    double kP = .03;
    double kI = 0; 
    double kD = 0;

    @Override
    public void initialize() {
        // drivetrain.applyRequest(() -> {
        //     return new SwerveRequest.RobotCentric()
        //             .withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        // }).schedule();
        IsDone = false;
        pid = new PIDController(kP, kI, kD);
        pid.setSetpoint(TargetDegree);
    }

    boolean IsDone = false;
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        
    @Override
    public void execute() {
        // if (IsDone) {
        //     return;
        // }
            double RotateSpeed = 0;

            double CurrentDegree = drivetrain.getPose().getRotation().getDegrees();

            double CurrentError = CurrentDegree - TargetDegree;
            double ErrorDegree = Math.abs(CurrentError);

            if (ErrorDegree < ErrorRate) {
                IsDone = true;
                RotateSpeed = 0;
            } else {
                RotateSpeed = pid.calculate(CurrentDegree);
            }
            final double FinalRotateSpeed = RotateSpeed * MaxAngularRate;
            System.out.println("ErrorDegree " + ErrorDegree);
            System.out.println("RotateSpeed " + FinalRotateSpeed);
            System.out.println("CurrentDegree " + CurrentDegree);

            

            drivetrain.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(0).withVelocityY(0).withRotationalRate(FinalRotateSpeed * MaxAngularRate)
                    .withRotationalDeadband(0));

        // drivetrain.applyRequest(() -> {
        //     return new SwerveRequest.RobotCentric()
        //             .withVelocityX(0).withVelocityY(0).withRotationalRate(FinalRotateSpeed * MaxAngularRate)
        //             .withRotationalDeadband(0);
        // });
    }

    @Override
    public boolean isFinished() {
        return IsDone;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
