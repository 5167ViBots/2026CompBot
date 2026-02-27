// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSubsystem extends SubsystemBase {
  CANdle candle;
  Timer lightTimer;
  /** Creates a new ExampleSubsystem. */
  public LightSubsystem() {
    candle = new CANdle(0);
    
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.RGB;
    var chud = candle.getConfigurator();
    chud.apply(config);
    lightTimer = new Timer();
  }

  public void startTimer(){
    // lightTimer.reset();
    lightTimer.start();
  }

  public void setColor(Color InColor) {
    SolidColor Config = new SolidColor(0,7);

    Config.Color = RGBWColor.fromHex(InColor.toHexString()).get();
    candle.setControl(Config);
  }

  public void resetTimer(){
    lightTimer.stop();
    lightTimer.reset();
  }

  public void strobeColor(Color inColor, double frequency) {
        StrobeAnimation Config2 = new StrobeAnimation(0, 7);
        Config2.Color = RGBWColor.fromHex(inColor.toHexString()).get();
        Config2.FrameRate = frequency;
        candle.setControl(Config2);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double timerSeconds = lightTimer.get();

    // if (timerSeconds == 0){
    //   setColor(Color.kWhite);
    // }
    if ((timerSeconds > 1) && (timerSeconds < 5)) {
      setColor(Color.kGreen);
      System.out.println(timerSeconds);
    }
    else if ((timerSeconds > 5) && (timerSeconds < 10)) {
      setColor(Color.kYellow);
      System.out.println(timerSeconds);
    }
    else if ((timerSeconds > 10) && (timerSeconds < 20)) {
      setColor(Color.kRed);
      System.out.println(timerSeconds);
    } 
    else if (timerSeconds > 20) {
      strobeColor(Color.kYellow, 2);
      System.out.println(timerSeconds);
    }
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
