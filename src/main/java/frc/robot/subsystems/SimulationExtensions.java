// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TurretSubsystem;
//import frc.robot.subsystems.HoodSubsystem;

public class SimulationExtensions extends SubsystemBase {

    private final TurretSubsystem turret;
    //private final HoodSubsystem hood;

    // Mechanism objects
    private Mechanism2d turretMech;
    private MechanismLigament2d turretArm;

    private Mechanism2d hoodMech;
    private MechanismLigament2d hoodArm;

    public SimulationExtensions(TurretSubsystem turret) {
        this.turret = turret;
        //this.hood = hood;

        if (!RobotBase.isSimulation()) {
            return;  // skip everything on real robot
        }

        // Turret visualizer
        turretMech = new Mechanism2d(60, 60);
        MechanismRoot2d turretRoot = turretMech.getRoot("Turret Root", 30, 30);
        turretArm = turretRoot.append(
            new MechanismLigament2d("Turret Arm", 40, 0, 6, new Color8Bit(Color.kRed))
        );
        SmartDashboard.putData("Turret Viz", turretMech);

        // Hood visualizer
        hoodMech = new Mechanism2d(60, 60);
        MechanismRoot2d hoodRoot = hoodMech.getRoot("Hood Root", 30, 10);
        hoodArm = hoodRoot.append(
            new MechanismLigament2d("Hood Arm", 35, 0, 8, new Color8Bit(Color.kBlue))
        );
        SmartDashboard.putData("Hood Viz", hoodMech);
    }

    @Override
    public void periodic() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        if (turretArm != null) {
            double turretAngleDeg = turret.getPositionDegrees();  // turret angle method
            turretArm.setAngle(turretAngleDeg);
        }

        if (hoodArm != null) {
          //  double hoodAngleDeg = hood.getPositionDegrees();  // hood angle method
           // hoodArm.setAngle(hoodAngleDeg);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Optional: extra sim-only updates (e.g., more detailed visuals)
    }
}