// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class CandleSubsystem extends SubsystemBase {

    public final CANdle candle;

    private Color lastColor = Color.kBlack;
    private boolean isStrobe = false;
    private double lastStrobeHz = 0.0;

        /** Creates a new CandleSubsystem. */
        public CandleSubsystem() {

        candle = new CANdle(Constants.CandleConstants.CANDLE_DEVICE_ID, Constants.CandleConstants.CANDLE_CANBUS);
        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.StripType = StripTypeValue.RGB;
        cfg.LED.BrightnessScalar = 1.0;
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);
        
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
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public void setOff(){
        // set candle to OFF
        if (!lastColor.equals(Color.kBlack)){
            candle.setControl(new SolidColor( 0,8)
                .withColor(new RGBWColor(0, 0, 0, 0)));
            lastColor = Color.kBlack;
        }
    }
    
    public void setColor(Color targetColor){

        if ((!lastColor.equals(targetColor)) || isStrobe) {                   // if the color, strobeishness, or strobe speed change, turn off then reprogram CANdle.

        // "Color" is type "double" with a range 0-1. We need to convert to (int) 0-255 for RGBWColor.
            int r = (int) Math.round(targetColor.red    * 255.0);
            int g = (int) Math.round(targetColor.green  * 255.0);
            int b = (int) Math.round(targetColor.blue   * 255.0);
            int w = 0;

            lastColor = targetColor;

            candle.setControl(new SolidColor( 0,8) // set all LED to the same color
                .withColor(new RGBWColor(r, g, b, w))); 
        }
    }

    public void setStrobe(Color targetColor, double speedHz) {

        if ( !lastColor.equals(targetColor) || !isStrobe || lastStrobeHz != speedHz) {         // if the color, strobeishness, or strobe speed change, turn off then reprogram CANdle.

            int r = (int) Math.round(targetColor.red    * 255.0);
            int g = (int) Math.round(targetColor.green  * 255.0);
            int b = (int) Math.round(targetColor.blue   * 255.0);
            int w = 0;

            candle.setControl(new StrobeAnimation(0, 8) // All LEDs flash simultaneously
                .withFrameRate(speedHz)                                 // set framerate
                .withColor(new RGBWColor(r, g, b, w)));                 // set color
        }
    }

    public void defaultBehavior() {
        Color defaultColor = Constants.CandleConstants.DEFAULT_COLOR;
        if (DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().get() == Alliance.Red)
            {
                defaultColor = Constants.CandleConstants.RED_ALLIANCE_COLOR;
            } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                defaultColor = Constants.CandleConstants.BLUE_ALLIANCE_COLOR;
            } else {
                defaultColor = Constants.CandleConstants.DEFAULT_COLOR;
            }
        }
        setColor(defaultColor);
    }
}