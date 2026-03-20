// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;
// import frc.robot.Constants;
// import frc.robot.ShuffleboardControl;

// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.util.datalog.DoubleLogEntry;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.ShuffleboardControl;

// public class HoodSubsystem extends SubsystemBase {
  
//   public TalonFX hoodMotor;

//   private final MotionMagicVoltage hoodMotionMagicRequest = new MotionMagicVoltage(0);
//   private double currentTargetDegrees = 0;

//   /** Creates a new HoodSubsystem. */
//   public HoodSubsystem() {
//     hoodMotor = new TalonFX(Constants.HoodConstants.hoodMotorID, Constants.HoodConstants.hoodMotorCANBus);

//     TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
//     hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     hoodConfig.CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.HOOD_CURRENT_LIMIT;
//     hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//     hoodConfig.Feedback.FeedbackRemoteSensorID = 0;
//     hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;


//     hoodConfig.MotorOutput.Inverted = Constants.HoodConstants.INVERT_MOTOR ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

//     hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
//     hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.HoodConstants.UPPER_LIMIT;
//     hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//     hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.HoodConstants.LOWER_LIMIT;

//     // PID Control (motion magic)

//     hoodConfig.Slot0 = new Slot0Configs()
//         .withKP(Constants.HoodConstants.kP)
//         .withKI(Constants.HoodConstants.kI)
//         .withKD(Constants.HoodConstants.kD);
      
//     hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.HoodConstants.MOTION_CRUISE_VELOCITY / 360 * Constants.HoodConstants.GEAR_RATIO; // convert from degrees/s to ticks/s
//     hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.HoodConstants.MOTION_ACCELERATION / 360 * Constants.HoodConstants.GEAR_RATIO; // convert from degrees/s^2 to ticks/s^2

//     hoodMotor.getConfigurator().apply(hoodConfig);

//   }

//   public void setPositionDegrees(double positionDegrees) {
//     double positionRotaton = degreesToMotorRevolutions(positionDegrees);

//     double angleRad = Math.toRadians(positionDegrees);
//     double gravityFF = Constants.HoodConstants.kG * Math.cos(angleRad);

//     gravityFF = Math.min(Math.max(gravityFF, -Constants.HoodConstants.FF_CAP), Constants.HoodConstants.FF_CAP); // cap the feedforward to prevent excessive values

//     hoodMotionMagicRequest.Position = positionRotaton;
//     hoodMotionMagicRequest.FeedForward = gravityFF;

//     hoodMotor.setControl(hoodMotionMagicRequest);
//     currentTargetDegrees = positionDegrees;
//   }

//   public double degreesToMotorRevolutions(double degrees) {

//     double revolutionsPerDegree = (Constants.HoodConstants.UPPER_LIMIT - Constants.HoodConstants.LOWER_LIMIT) / (60 - 40);

//     return degrees * revolutionsPerDegree;
//   }

//   public double revolutionsToDegrees(double revolutions) {
//     double degreesPerRevolution = (60 - 40) / (Constants.HoodConstants.UPPER_LIMIT - Constants.HoodConstants.LOWER_LIMIT);
//     return revolutions * degreesPerRevolution;
//   }

//   public boolean atTargetPosition() {
//     double currentPositionRotation = degreesToMotorRevolutions(getPositionDegrees());
//     double targetPositionRotation = degreesToMotorRevolutions(getTargetPositionDegrees());
//     double toleranceRotation = degreesToMotorRevolutions(Constants.HoodConstants.POSITION_TOLERANCE_DEGREES);
//     return Math.abs(currentPositionRotation - targetPositionRotation) <= toleranceRotation;
//   }

//   public double getPositionDegrees() {
//     // Convert encoder rotations to degrees
//     return revolutionsToDegrees(hoodMotor.getPosition().getValueAsDouble());
//   }

//   public double getTargetPositionDegrees() {
//     return currentTargetDegrees;
//   }

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }

//   public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) {
//     Slot0Configs slot0 = new Slot0Configs().withKP(kp).withKI(ki).withKD(kd).withKV(kv).withKG(kg);
//     TalonFXConfiguration config = new TalonFXConfiguration();
//     config.Slot0 = slot0;
//     config.MotionMagic.MotionMagicCruiseVelocity = mmv / 360 * Constants.HoodConstants.GEAR_RATIO;
//     config.MotionMagic.MotionMagicAcceleration = mma / 360 * Constants.HoodConstants.GEAR_RATIO;
//     hoodMotor.getConfigurator().apply(config);
//   }

// }
