// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.ShuffleboardControl;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;


public class IntakeExtender extends SubsystemBase {

    // motors for intake extender
    private final TalonFX leaderMotor;      // left
    private final TalonFX followerMotor;    // right

    // MotionMagic control
    // private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private double currentTargetTicks = 0.0; // for gravity compensation, monitor in periodic() to make sure extender isn't sagging

    // "Slew rate" limits jerk, improves smoothness of motion
    // private final SlewRateLimiter outputLimiter = new SlewRateLimiter(Constants.IntakeExtenderConstants.SLEW_RATE_LIMITER); // 3 units per second (optional)

    // target setpoint for the extender position (start up)
    private double targetPositionDegrees = Constants.IntakeExtenderConstants.UP_POSITION;

    public IntakeExtender() {
        // initialize motors
        leaderMotor = new TalonFX(Constants.IntakeExtenderConstants.leaderExtenderID, Constants.IntakeExtenderConstants.leaderExtenderCANBus);
        followerMotor = new TalonFX(Constants.IntakeExtenderConstants.followerExtenderID, Constants.IntakeExtenderConstants.followerExtenderCANBus);

        // configure leader
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.Feedback.FeedbackRemoteSensorID = Constants.IntakeExtenderConstants.LEFT_ENCODER_DIO;
        leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        leaderConfig.MotorOutput.PeakForwardDutyCycle = 0.25;
        leaderConfig.MotorOutput.PeakReverseDutyCycle = -0.25;

        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // We don't want the motor to move when it loses power

        // PID Control (Motion Magic)
        leaderConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.IntakeExtenderConstants.kP)
            .withKI(Constants.IntakeExtenderConstants.kI)
            .withKD(Constants.IntakeExtenderConstants.kD);
        // leaderConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeExtenderConstants.MOTION_CRUISE_VELOCITY;
        // leaderConfig.MotionMagic.MotionMagicAcceleration= Constants.IntakeExtenderConstants.MOTION_ACCELERATION;

        leaderMotor.getConfigurator().apply(leaderConfig);

        // configure follower
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.Feedback.FeedbackRemoteSensorID = Constants.IntakeExtenderConstants.RIGHT_ENCODER_DIO;
        followerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(),MotorAlignmentValue.Opposed));

        followerMotor.getConfigurator().apply(followerConfig);


        // initialize motion magic for gravity compensation
        // motionMagicRequest.Position = 0.0;
        // motionMagicRequest.FeedForward = 0.0;

    }

    // move the extender up/down
    public void setPositionDegrees(double positionDegrees) {

        if (leaderMotor == null){
            System.out.println("leaderMotor is null!");
            return;
        }
        // if (motionMagicRequest == null){
        //     System.out.println("motionMagicRequest is null!");
        //     return;
        // }

        // targetPositionDegrees = positionDegrees;

        // double targetTicks = (positionDegrees / 360.0) * Constants.IntakeExtenderConstants.IntakeGearRatio * Constants.IntakeExtenderConstants.ENCODER_TICKS_PER_REVOLUTION;
        // currentTargetTicks = targetTicks;

        // do the motion magic stuff.
        // motionMagicRequest.Position = targetTicks;
        // motionMagicRequest.FeedForward = calculateGravityFF(); // gravity compensation
        // leaderMotor.setControl(motionMagicRequest);

        leaderMotor.setControl(new PositionDutyCycle(positionDegrees));
    }

    public void extenderUp() {
        setPositionDegrees(Constants.IntakeExtenderConstants.UP_POSITION);
        System.out.println("Extender Up");
    }

    public void extenderDown() {
        setPositionDegrees(Constants.IntakeExtenderConstants.DOWN_POSITION);
        System.out.println("ExtenderDown");
    }

    public boolean atTargetPosition() {
        return Math.abs(getPositionDegrees() - targetPositionDegrees) <= Constants.IntakeExtenderConstants.POSITION_TOLERANCE_DEGREES;
    }

    public boolean atUpPosition() {
        return (leaderMotor.getPosition().getValueAsDouble() > 0.29);
    }

    public boolean atDownPosition() {
        return ((targetPositionDegrees == Constants.IntakeExtenderConstants.DOWN_POSITION) && atTargetPosition());
    }

    public double getPositionDegrees() {
        double motorTicks = leaderMotor.getPosition().getValueAsDouble();
        return (motorTicks / (Constants.IntakeExtenderConstants.IntakeGearRatio * Constants.IntakeExtenderConstants.ENCODER_TICKS_PER_REVOLUTION)) * 360.0;
    }

    // compute gravity feed forward value in volts
    // public double calculateGravityFF(){
    //     double currentDegrees = getPositionDegrees();
    //     double currentRadians = Math.toRadians(currentDegrees);
    //     return Constants.IntakeExtenderConstants.kG * Math.cos(currentRadians); // when at 90 degrees, cos(90) = 0, so there's no feedforward compensation. When at 0 degrees, maximum feed forward.
    // }

    public void stop() {
        setPower(0);
        // leaderMotor.stopMotor();
        targetPositionDegrees = getPositionDegrees();
        currentTargetTicks = (targetPositionDegrees/360.0) * Constants.IntakeExtenderConstants.IntakeGearRatio * Constants.IntakeExtenderConstants.ENCODER_TICKS_PER_REVOLUTION;
        // System.out.println("Intake Extender stopped at "+targetPositionDegrees+" degrees");
    }

    // void setPositionTicks(double motorTicks){

    //     // System.out.println("setPositionTicks is called with " + motorTicks);

    //     if (leaderMotor == null){
    //         System.out.println("leaderMotor is null!");
    //         return;
    //     }

    //     if (motionMagicRequest == null){
    //         System.out.println("motionMagicRequest is null!");
    //         return;
    //     }
    //     currentTargetTicks = motorTicks;
    //     motionMagicRequest.Position = motorTicks;
    //     motionMagicRequest.FeedForward = calculateGravityFF();
    //     leaderMotor.setControl(motionMagicRequest);
    // }

    double getPositionTicks(){
        return leaderMotor.getPosition().getValueAsDouble();
    }

    void setPower(double power){
        leaderMotor.setControl(new DutyCycleOut(power));
    }

    double getPower(){
        return leaderMotor.getDutyCycle().getValueAsDouble();
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
        // double gravityFF = calculateGravityFF();

        // if (targetPositionDegrees != Double.NaN){
        //     motionMagicRequest.FeedForward = gravityFF;
        //     motionMagicRequest.Position = currentTargetTicks;
        //     leaderMotor.setControl(motionMagicRequest);
        // }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) {
        Slot0Configs slot0 = new Slot0Configs().withKP(kp).withKI(ki).withKD(kd).withKV(kv).withKG(kg);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = slot0;
        config.MotionMagic.MotionMagicCruiseVelocity = mmv;
        config.MotionMagic.MotionMagicAcceleration = mma;
        leaderMotor.getConfigurator().apply(config);
  }

}
