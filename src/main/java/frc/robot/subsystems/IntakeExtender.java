package frc.robot.subsystems;
import frc.robot.Constants;
//import frc.robot.ShuffleboardControl;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
 TalonFX motor1; //left motor
 TalonFX motor2; //right motor

 private final DutyCycleEncoder leaderEncoder;
 private final DutyCycleEncoder followerEncoder;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private double currentTargetTicks = 0.0;

     private double targetPositionDegrees = Constants.IntakeExtenderConstants.UP_POSITION_DEGREES;


  public IntakeExtender() {

  motor1 = new TalonFX(Constants.IntakeExtenderConstants.leaderExtenderID, Constants.IntakeExtenderConstants.leaderExtenderCANBus);
  motor2 = new TalonFX(Constants.IntakeExtenderConstants.followerExtenderID, Constants.IntakeExtenderConstants.followerExtenderCANBus);

 leaderEncoder = new DutyCycleEncoder(Constants.IntakeExtenderConstants.LEFT_ENCODER_DIO);
 followerEncoder = new DutyCycleEncoder(Constants.IntakeExtenderConstants.RIGHT_ENCODER_DIO);
  
 TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
  leaderConfig.MotorOutput.Inverted = Constants.IntakeExtenderConstants.INVERT_LEADER_MOTOR ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
  leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // We don't want the motor to move when it loses power
  leaderConfig.CurrentLimits = new CurrentLimitsConfigs()
    .withStatorCurrentLimit(Constants.IntakeExtenderConstants.MAX_CURRENT_AMPS)
    .withStatorCurrentLimitEnable(true);

   // PID Control (with Motion Magic)
        leaderConfig.Slot0 = new Slot0Configs()
            .withKP(Constants.IntakeExtenderConstants.kP)
            .withKI(Constants.IntakeExtenderConstants.kI)
            .withKD(Constants.IntakeExtenderConstants.kD)
            .withKV(Constants.IntakeExtenderConstants.kV)
            .withKG(Constants.IntakeExtenderConstants.kG);
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeExtenderConstants.MOTION_CRUISE_VELOCITY;
        leaderConfig.MotionMagic.MotionMagicAcceleration= Constants.IntakeExtenderConstants.MOTION_ACCELERATION;

        motor1.getConfigurator().apply(leaderConfig);

         // configuring follower
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.Inverted = Constants.IntakeExtenderConstants.INVERT_FOLLOWER_MOTOR ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerConfig.CurrentLimits = leaderConfig.CurrentLimits;

        motor1.setControl(new Follower(motor1.getDeviceID(),MotorAlignmentValue.Opposed));

        motor1.getConfigurator().apply(followerConfig);

        // initialize motion magic
        motionMagicRequest.Position = 0.0;
        motionMagicRequest.FeedForward = 0.0;

        resetPositionFromEncoders();
  }

   public void resetPositionFromEncoders(){
        double leftPos = leaderEncoder.get();
        double rightPos = followerEncoder.get();

        double averagePos = (leftPos + rightPos) / 2.0;

        double diffDegrees = Math.abs(leftPos - rightPos) * 360; // convert to degrees
        if (diffDegrees > 5.0) {
            System.out.println("Warning: Intake Extender encoder disagreement: " + diffDegrees + " degrees");
        }

        double motorTicks = averagePos * Constants.IntakeExtenderConstants.IntakeGearRatio * Constants.IntakeExtenderConstants.ENCODER_TICKS_PER_REVOLUTION;

        motor1.setPosition(motorTicks);
    }

     public void setPositionDegrees(double positionDegrees) {

        if (motor1 == null){
            System.out.println("leaderMotor is null!");
            return;
        }
        if (motionMagicRequest == null){
            System.out.println("motionMagicRequest is null!");
            return;
        }

        targetPositionDegrees = positionDegrees;

        double targetTicks = (positionDegrees / 360.0) * Constants.IntakeExtenderConstants.IntakeGearRatio * Constants.IntakeExtenderConstants.ENCODER_TICKS_PER_REVOLUTION;
        currentTargetTicks = targetTicks;

        // do the motion magic stuff.
        motionMagicRequest.Position = targetTicks;
        motionMagicRequest.FeedForward = calculateGravityFF(); // gravity compensation
        motor1.setControl(motionMagicRequest);
    }

     public void extenderUp() {
        setPositionDegrees(Constants.IntakeExtenderConstants.UP_POSITION_DEGREES);
    }

    public void extenderDown() {
        setPositionDegrees(Constants.IntakeExtenderConstants.DOWN_POSITION_DEGREES);
    }

    public boolean atTargetPosition() {
        return Math.abs(getPositionDegrees() - targetPositionDegrees) <= Constants.IntakeExtenderConstants.POSITION_TOLERANCE_DEGREES;
    }

     public double getPositionDegrees() {
        double motorTicks = motor1.getPosition().getValueAsDouble();
        return (motorTicks / (Constants.IntakeExtenderConstants.IntakeGearRatio * Constants.IntakeExtenderConstants.ENCODER_TICKS_PER_REVOLUTION)) * 360.0;
    }

    // compute gravity feed forward value in volts
    public double calculateGravityFF(){
        double currentDegrees = getPositionDegrees();
        double currentRadians = Math.toRadians(currentDegrees);
        return Constants.IntakeExtenderConstants.kG * Math.cos(currentRadians); // when at 90 degrees, cos(90) = 0, so there's no feedforward compensation. When at 0 degrees, maximum feed forward.
    }

    public void stop() {
        motor1.stopMotor();
        targetPositionDegrees = getPositionDegrees();
        currentTargetTicks = (targetPositionDegrees/360.0) * Constants.IntakeExtenderConstants.IntakeGearRatio * Constants.IntakeExtenderConstants.ENCODER_TICKS_PER_REVOLUTION;
        System.out.println("Intake Extender stopped at "+targetPositionDegrees+" degrees");
    }

    void setPositionTicks(double motorTicks){

        // System.out.println("setPositionTicks is called with " + motorTicks);

        if (motor1 == null){
            System.out.println("leaderMotor is null!");
            return;
        }

        if (motionMagicRequest == null){
            System.out.println("motionMagicRequest is null!");
            return;
        }
        currentTargetTicks = motorTicks;
        motionMagicRequest.Position = motorTicks;
        motionMagicRequest.FeedForward = calculateGravityFF();
        motor1.setControl(motionMagicRequest);
    }

    double getPositionTicks(){
        return motor1.getPosition().getValueAsDouble();
    }

    void setPower(double power){
        motor1.setControl(new DutyCycleOut(power));
    }

    double getPower(){
        return motor1.getDutyCycle().getValueAsDouble();
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
    double gravityFF = calculateGravityFF();

    if (targetPositionDegrees != Double.NaN){
        motionMagicRequest.FeedForward = gravityFF;
        motionMagicRequest.Position = currentTargetTicks;
        motor1.setControl(motionMagicRequest);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

   public void applyPid(double kp, double ki, double kd) {
        Slot0Configs slot0 = new Slot0Configs().withKP(kp).withKI(ki).withKD(kd);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0 = slot0;
        motor1.getConfigurator().apply(config);
  }
}
