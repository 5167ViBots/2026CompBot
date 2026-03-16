// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

// Commands (these can all be called by frc.robot.commands.*)

import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.RotateBotToDegreeCommand;
import frc.robot.commands.SetVerticalIndexStateCommand;
import frc.robot.commands.ShooterAuto;
import frc.robot.commands.ShooterContinuousCommand;
import frc.robot.commands.ShooterStartCommand;
import frc.robot.commands.ShooterStopCommand;
import frc.robot.commands.StaticAimCommand;
import frc.robot.commands.ToggleHorizontalIndexCommand;
import frc.robot.commands.ToggleVerticalIndexCommand;
import frc.robot.commands.IndexToShooterCommand;
import frc.robot.commands.IndexVertical;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.IntakeExtenderAuton;
import frc.robot.commands.IntakeExtenderUp;
import frc.robot.commands.IntakeExtenderUpAuto;
import frc.robot.commands.IntakeExtenderDown;
import frc.robot.Constants.BoundaryConstants;
import frc.robot.Constants.CandleConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.ShuffleboardControl.MotorAccessor;
import frc.robot.commands.IndexReverseCommand;
import frc.robot.commands.IndexStopCommand;
import frc.robot.commands.AdvancedRotateBotToDegreeCommand;
import frc.robot.commands.CANdleSetColorCommand;
import frc.robot.commands.DynamicAimCommand;
import frc.robot.commands.IndexAuto;
import frc.robot.commands.IndexHorizontal;
import frc.robot.commands.StaticAimCommand;

import frc.robot.generated.TunerConstants;

// Subsystems (these can all be called by frc.robot.subsystems.*)

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeExtender;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimulationExtensions;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.CandleSubsystem;

import frc.robot.BoundaryManager;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// import frc.robot.Constants;

public class RobotContainer {

    // Subsystems
        HoodSubsystem hood;
        IndexSubsystem index;
        IntakeExtender intakeExtender;
        IntakeSubsystem intake;
        ShooterSubsystem shooter;
        TurretSubsystem turret;
        // CandleSubsystem candle;
        // ClimbSubsystem climber;

    // limelight subsystem & initialize timestamp
        private LimelightSubsystem limelightFront;
        private LimelightSubsystem limelightSide;
        private double lastVisionTimestamp = 0;


    // initialize Robot State & Robot Field Position
        private Constants.robotStates.State currentState = Constants.robotStates.State.CALIBRATION;
        private FieldConstants.FieldRegion currentRegion = FieldConstants.FieldRegion.UNKNOWN;

    // Limits on how fast the robot can move & turn
        private double MaxSpeed = BoundaryConstants.MaxSpeed * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

       // private final CommandXboxController joystick = new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
       private final CommandPS5Controller joystick = new CommandPS5Controller(Constants.OperatorConstants.kDriverControllerPort);

        private final CommandJoystick buttonBoard = new CommandJoystick(Constants.OperatorConstants.kButtonBoardPort);
        private final CommandJoystick buttonBoardJR = new CommandJoystick(2);
        public final CommandSwerveDrivetrain drivetrain;

    // creates a Boundary manager to prevent the robot from running into things (currently unused, see commented-out section in drive default)
        private final BoundaryManager boundaryManager = new BoundaryManager();

    // Fields for Robot State tab in Shuffleboard
        private static final ShuffleboardTab robotStateTab = Shuffleboard.getTab("Robot State");

        private GenericEntry robotStateEntry;
        private GenericEntry robotFieldEntry;
        private GenericEntry robotDriveEntry;
        private GenericEntry robotPoseEntry;
        private GenericEntry turretAngleDisplay;
        private GenericEntry hoodAngleDisplay;

    // Robot state triggers (necessary for defining commands later)

        public Trigger fireOnMoveTrigger() {
            return new Trigger(() -> currentState == Constants.robotStates.State.FIRE_ON_THE_MOVE);
        }

        public Trigger pushTrigger() {
            return new Trigger(() -> currentState == Constants.robotStates.State.PUSH);
        }

        public Trigger climbTrigger() {
            return new Trigger(() -> currentState == Constants.robotStates.State.CLIMB);
        }

        public Trigger idleTrigger() {
            return new Trigger(() -> currentState == Constants.robotStates.State.IDLE);
        }

        public Trigger stopAndShootTrigger() {
            return new Trigger(() -> currentState == Constants.robotStates.State.STOP_AND_SHOOT);
        }

        public Trigger calibrationTrigger() {
            return new Trigger(() -> currentState == Constants.robotStates.State.CALIBRATION);
        }

    // Linking "fire on move" and "stop and shoot" as these have similar commands

        public Trigger isShootingTrigger() {
            return fireOnMoveTrigger().or(stopAndShootTrigger());
        }

    // Unjam button on Buttonboard
        public Trigger unJamActive = buttonBoard.button(Constants.OperatorConstants.UNJAM_BUTTON);

    // Climb buttons

        public Trigger climbRaiseArmTrigger = buttonBoard.button(Constants.OperatorConstants.CLIMB_RAISE_ARM_BUTTON);
        public Trigger climbLowerArmTrigger = buttonBoard.button(Constants.OperatorConstants.CLIMB_LOWER_ARM_BUTTON);

        // removed these triggers as we were only using them to check if the climber hook & intake extender were both extended at the same time (illegal)
        // Keeping them here for posterity, and in case we want to use them later. Bute note that this will cause a crash. Better to declare here:
            // e.g. "public Trigger intakeExtenderUpTrigger;"
            // and later initialize them in configureBindings() { ... intakeExtenderUpTrigger = new Trigger(intakeExtender::atUpPosition)}
        // public Trigger intakeExtenderUpTrigger = new Trigger(intakeExtender::atUpPosition);
        // public Trigger intakeExtenderDownTrigger = new Trigger(intakeExtender::atDownPosition);

    // Fire button on buttonboard
    public Trigger fireButton() {
        return buttonBoard.button(Constants.OperatorConstants.FIRE_BUTTON);
    }

    // Trigger to detect if the Turret is way out of line. We will use this to disable the indexer to prevent shooting while turret is reorienting
    private Trigger turretIsReorienting = new Trigger (() -> {
        double errorDeg = Math.abs(turret.getPositionDegrees() - turret.getTargetPositionDegrees());
        return errorDeg > TurretConstants.LARGE_ERROR;
    });

    private boolean isRobotRelative = false; // boolean to switch between fixed/field centric (false) and relative/robot centric (true)

    public RobotContainer() {

        // Instantiate subsystems.
        drivetrain = TunerConstants.createDrivetrain();
        intake          = new IntakeSubsystem();
        shooter         = new ShooterSubsystem();
        intakeExtender  = new IntakeExtender();
        index           = new IndexSubsystem();
        hood            = new HoodSubsystem();
        turret          = new TurretSubsystem();
        limelightFront       = new LimelightSubsystem("front");
        limelightSide      = new LimelightSubsystem("side");
        // candle          = new CandleSubsystem();
       // climber         = new ClimbSubsystem();

        // Set the robot position to x = 0.5, y = 0.5, rotation = 0 ONLY for simulation purposes
        if (RobotBase.isSimulation()) {
            new SimulationExtensions(turret, hood);
            drivetrain.resetPose(new Pose2d( 0.5, 0.5, new Rotation2d(0)));
        }
        setRobotState(Constants.robotStates.State.CALIBRATION);

        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();
        // Run configuration files to set up everything.
        configureBindings();
        configureDefaults();
        configureShuffleboard();
        registerPathPlannerCommands();
        registerAutons();


            // Timer.delay(0.5);
            // ShuffleboardControl.update();
            // Shuffleboard.update();

        // Default robot state. Temporarily moved from IDLE to FOM. Depending on operator preference we may want to set this differently.
    }

     private void registerPathPlannerCommands() {
         NamedCommands.registerCommand("Intake", new IntakeAuto(intake));  
         NamedCommands.registerCommand("IntakeStop", new IntakeStopCommand(intake)); 
         NamedCommands.registerCommand("IntakeExtend", new IntakeExtenderAuton(intakeExtender));  
         NamedCommands.registerCommand("IntakeRetract", new IntakeExtenderUpAuto(intakeExtender)); 
         NamedCommands.registerCommand("Shoot", new ShooterAuto(shooter));  
         NamedCommands.registerCommand("StopShooting", new ShooterStopCommand(shooter));
         NamedCommands.registerCommand("Index", new IndexAuto(index)); 
         NamedCommands.registerCommand("IndexStop", new IndexStopCommand(index));   
        //  NamedCommands.registerCommand("ClimbUp", new ClimberArmUpCommand(climber));  
        //  NamedCommands.registerCommand("ClimbDown", new ClimberArmDownCommand(climber));
        //  NamedCommands.registerCommand("ClimbStop", new ClimberArmStopCommand(climber));
         
        }

        public SendableChooser<Command> AutonChooser = new SendableChooser<Command>();
        //SendableChooser<Command> autochooser; 
    private void registerAutons() {
    
    //Create Shuffleboard Tab
    var tab = Shuffleboard.getTab("Auton");
    
    AutonChooser = AutoBuilder.buildAutoChooser("est Auto");

    //Register Auton modes
    // AutonChooser.addOption("Leave and Score","Leave and Score");
    // AutonChooser.addOption("auton","auton");
    
    
    // //Set the default Auton
    // AutonChooser.setDefaultOption("auton","auton");
    
    //Add to shuffleboard
    tab.add(AutonChooser);
  }

    // Set defaults for the various subsystems.
    private void configureDefaults() {

        drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {

                double leftY = joystick.getLeftY();
                double leftX = -joystick.getLeftX();
                double rightX = -joystick.getRightX();

                double deadband = 0.12;                             // added a deadband because my joysticks do not default to 0/0/0
                leftY = Math.abs(leftY) > deadband ? leftY : 0;
                leftX = Math.abs(leftX) > deadband ? leftX : 0;
                rightX = Math.abs(rightX) > deadband ? rightX : 0;

                double vx = -leftX * MaxSpeed;
                double vy = leftY * MaxSpeed;
                double omega = rightX * MaxAngularRate;

                if(isRobotRelative) {
                    return new SwerveRequest.RobotCentric()
                        .withVelocityX(-vy)
                        .withVelocityY(-vx)
                        .withRotationalRate(omega);
                    } else {
                    return new SwerveRequest.FieldCentric()
                        .withVelocityX(vy)
                        .withVelocityY(vx)
                        .withRotationalRate(omega);
                }    

                // This section is used for limiting the robot's ability to drive into/near obstacles (HUB, Walls, etc.)
                // having some issues with the robot getting stuck inside obstacles during sim, we can revisit boundaries when physical robot is available

                // ChassisSpeeds raw = new ChassisSpeeds(vx, vy, omega);

                // ChassisSpeeds safe = boundaryManager.constrainSpeeds(drivetrain.getPose(), raw);

                // return drive.withVelocityX(safe.vxMetersPerSecond * MaxSpeed) // Drive forward with negative Y (forward)
                //     .withVelocityY(safe.vyMetersPerSecond * MaxSpeed) // Drive left with negative X (left)
                //     .withRotationalRate(safe.omegaRadiansPerSecond * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );


        // intake.setDefaultCommand(
        //     Commands.run(intake::stop, intake)
        // ); // default to stopped when not called

        // shooter.setDefaultCommand(
        //     Commands.run(shooter::stop, shooter)
        // ); // default to stopped when not called

        intakeExtender.setDefaultCommand(
            Commands.run(intakeExtender::stop, intakeExtender)
        ); // default to stopped when not called

        // index.setDefaultCommand(
        //     Commands.run(index::stop, index)
        // ); // default to up position when not called

        turret.setDefaultCommand(
            Commands.run(turret::stop, turret)
        ); // stops turret from moving when not called.

        // climber.setDefaultCommand(
        //     Commands.parallel(
        //         Commands.run(climber::stopArm, climber)
        //     ));
    }

    // this section binds commands to triggers. VERY IMPORTANT for general operation and ESPECIALLY IMPORTANT for state-based.

    private void configureBindings() {
        // Note that X is defined as forward (blue -> Red) according to WPILib convention,
        // and Y is defined as blue-left according to WPILib convention.

        // Do not drive while the robot is in IDLE state. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // These are default commands added in the swerve module. We'll see if the driver likes/wants them

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // added a test command to aim turret with B buttojn on xboxcontroller
        // joystick.b().whileTrue(new DynamicAimCommand(turret, hood, this));

        // more default commands from Swerve.
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // default command from swerve
        // Reset the field-centric heading on left bumper press.
        // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // X button to enable intake motor. Unnecessary as this will be handled by the software states
        // joystick.x().whileTrue(new IntakeForwardCommand(intake));

        // Bind buttons to change robot states
        buttonBoard.button(Constants.OperatorConstants.FIRE_ON_THE_MOVE_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.FIRE_ON_THE_MOVE)));
        buttonBoard.button(Constants.OperatorConstants.PUSH_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.PUSH)));
        buttonBoard.button(Constants.OperatorConstants.CLIMB_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.CLIMB)));
        buttonBoard.button(Constants.OperatorConstants.IDLE_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.IDLE)));
        buttonBoard.button(Constants.OperatorConstants.STOP_AND_SHOOT_BUTTON).onTrue(Commands.runOnce(() -> setRobotState(Constants.robotStates.State.STOP_AND_SHOOT)));
        buttonBoardJR.button(6).onTrue(new IntakeExtenderUp(intakeExtender)); 
        buttonBoardJR.button(4).onTrue(new IntakeExtenderDown(intakeExtender));
        buttonBoardJR.button(3).whileTrue(new IntakeForwardCommand(intake));
        
        //\buttonBoardJR.button(1).whileTrue(getPathfindingCommand(2.4384, 2.4384, 180+45).andThen(new RotateBotToDegreeCommand(-135, 5, drivetrain)));
        buttonBoardJR.button(1).onTrue(new AdvancedRotateBotToDegreeCommand(-45, 15, drivetrain));



        buttonBoardJR.button(7).whileTrue(new ToggleHorizontalIndexCommand(index));
        // buttonBoardJR.button(5).whileTrue(new IndexVertical(index));
        buttonBoardJR.button(5).whileTrue(new SetVerticalIndexStateCommand(index,true));
        buttonBoardJR.button(5).whileFalse(new SetVerticalIndexStateCommand(index,false));

        buttonBoard.button(10).whileTrue(new ShooterStartCommand(shooter));



        // temporary for testing because phoenixTuner is being stupid.

        // isShooting pairs FOM and SAS
        // This trigger will activate when we first enter either the FIRE_ON_THE_MOVE or STOP_AND_SHOOT state, but not on subsequent scheduler runs while we're still in that state, due to the debounce.

        isShootingTrigger()

            // .and(isShootingTrigger().negate().debounce(0.05))
            .onTrue(new IntakeExtenderDown(intakeExtender));

        // // This trigger will activate when we leave the FIRE_ON_THE_MOVE state, but not on subsequent scheduler runs while we're still outside that state, due to the debounce.
        isShootingTrigger()
            // .and(isShootingTrigger().negate().debounce(0.05))
            .onFalse( 
            new IntakeExtenderUp(intakeExtender)
        );

        buttonBoard.button(Constants.OperatorConstants.STATIC_AIM_BUTTON).whileTrue(new StaticAimCommand(turret, hood, shooter, this));

        // climbTrigger().and(climbRaiseArmTrigger).whileTrue(new ClimberArmUpCommand(climber));
        // climbTrigger().and(climbLowerArmTrigger).whileTrue(new ClimberArmDownCommand(climber));

        // for fire on the  move, turn on intake, index, and shooter (unless we're unjamming)
        // If the turret is NOT reorienting, turn on index.

        // FIX SIMILAR TO STOPANDSHOOTTRIGGER + INTAKE/FIRE BUTTON

        fireOnMoveTrigger().and(unJamActive.negate()).and(turretIsReorienting.negate()).whileTrue(
            Commands.parallel(
                // new CANdleSetColorCommand(candle, CandleConstants.READY_TO_SHOOT_COLOR, CandleConstants.DEFAULT_STROBE_HZ), // flash "ready to shoot" (default green) while fire button is depressed
                new ShooterContinuousCommand(shooter),
                new IndexToShooterCommand(index),
                new IntakeForwardCommand(intake)
            )
        );

        // if the turret IS reorienting, turn off turret.
        fireOnMoveTrigger().and(unJamActive.negate()).and(turretIsReorienting).whileTrue(
            Commands.parallel(
                // new CANdleSetColorCommand(candle, CandleConstants.TURRET_AIMING, CandleConstants.NO_STROBE),                // set to "turret aiming" color (default purple) when we can't shoot
                new ShooterContinuousCommand(shooter),
                new IndexStopCommand(index),
                new IntakeForwardCommand(intake)
            )
        );

        fireOnMoveTrigger().whileTrue(new DynamicAimCommand(turret, hood, shooter, this));                                          // in "FOM" state we use dynamic aiming

        // for stop and shoot, if we're not unjamming or shooting, turn on the intake, turn off shooter & index.
        stopAndShootTrigger()
            .whileTrue(
            Commands.parallel(
                // new CANdleSetColorCommand(candle, Constants.CandleConstants.READY_TO_SHOOT_COLOR, Constants.CandleConstants.NO_STROBE),
                new ShooterContinuousCommand(shooter),
                new IntakeForwardCommand(intake),
                new IndexHorizontal(index)
            )
        );

        // for stop and shoot, if the fire button is being held, turn everything on

        // temporarily changed 

        // fireButton()
        //     .and(isShootingTrigger())
        //     .and(unJamActive.negate())
        //     .whileTrue(
        //     Commands.parallel(
        //         // new CANdleSetColorCommand(candle, Constants.CandleConstants.READY_TO_SHOOT_COLOR, Constants.CandleConstants.FAST_STROBE_HZ),
        //        // new IndexToShooterCommand(index))|
        //      //  new IndexVertical(index))
        //     new ShooterStartCommand(shooter))
        // );
 
        joystick.circle().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));

        //joystick.cross().onTrue(new IndexAuto(index));
        fireButton()
            // .and(isShootingTrigger())
            .and(unJamActive.negate())
            .whileTrue(
            Commands.parallel(
                // new CANdleSetColorCommand(candle, Constants.CandleConstants.READY_TO_SHOOT_COLOR, Constants.CandleConstants.FAST_STROBE_HZ),
              new IndexToShooterCommand(index)
               
            ));

        // repeatedly move index forward & reverse 
        // can't unjam while fire button is being pressed
        unJamActive.whileTrue( //.and(fireButton().negate()).and(isShootingTrigger())
            Commands.parallel(new IndexReverseCommand(index)          
            // new IntakeReverseCommand(intake)
           ));
            

        // Commands.repeatingSequence(
            //     new CANdleSetColorCommand(candle, CandleConstants.UNJAM_COLOR, CandleConstants.DEFAULT_STROBE_HZ),  // CANdle flashes while unjam is pressed
            //     new IndexReverseCommand(index).withTimeout(0.25),   // run index in reverse briefly to attempt to clear the jam
            //     Commands.runOnce(() -> index.stop()),                       // stop the motors to avoid damage
            //     Commands.waitSeconds(0.05),                         // wait a brief period while stopped
            //     new IndexToShooterCommand(index).withTimeout(0.25), // run index forward briefly to attempt to clear the jam after reversing
            //     Commands.runOnce(() -> index.stop()),
            //     Commands.waitSeconds(0.05)                          // wait 50 ms between each cycle of unjamming to allow motors to respond and potentially clear the jam
            // ).withTimeout(4.0)                                      // command will stop after 4 seconds even if the button is still held, to prevent potential damage from prolonged unjamming

        // while in PUSH state we're only activating the motors. Command above (isshooting) should make sure the intakeExtender moves up.
        pushTrigger().onTrue(
            Commands.parallel(
                // new CANdleSetColorCommand(candle, Constants.CandleConstants.PUSH_COLOR, Constants.CandleConstants.DEFAULT_STROBE_HZ),
                new ShooterStopCommand(shooter),
                new IndexStopCommand(index),
                new IntakeStopCommand(intake)
            )
        );

        // while in IDLE state we need to also deactivate drive
        idleTrigger().whileTrue(
            Commands.parallel(
                // new CANdleSetColorCommand(candle, Constants.CandleConstants.IDLE_COLOR, Constants.CandleConstants.NO_STROBE),
                new ShooterStopCommand(shooter),
                new IndexStopCommand(index),
                new IntakeStopCommand(intake),
                drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
            )
        );

        climbTrigger().whileTrue(
            Commands.parallel(
                // new CANdleSetColorCommand(candle, Constants.CandleConstants.CLIMB_COLOR, Constants.CandleConstants.NO_STROBE),
                new ShooterStopCommand(shooter),
                new IndexStopCommand(index),
                new IntakeStopCommand(intake)
                // Climb motors will be activated by separate triggers/buttons, not in this trigger
            )
        );

        calibrationTrigger().whileTrue(
            Commands.parallel(
                new IntakeStopCommand(intake),
                new IndexStopCommand(index)
                // new ShooterStopCommand(shooter)
            )
        );

        // toggle between fixed and relative drive:
        joystick.square()
            .onTrue(Commands.runOnce(() -> {
                isRobotRelative = !isRobotRelative;
            }));
    }

    // separate function for the sophisticated shuffleboard control so we can comment them out for competition (we don't want to be able to adjust PID once they're set)
    public void configureShuffleboard(){
        ShuffleboardControl.setupDashboard(currentState);

        // Display robot state - Fire on Move, Stop & Shoot, etc.
        robotStateEntry = robotStateTab
            .add("Robot State", "UNKNOWN")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();

        // Display which region the robot is in - Blue Left, Red Right, etc.
        robotFieldEntry = robotStateTab
            .add("Field Location", "UNKNOWN")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();
        
        robotDriveEntry = robotStateTab
            .add("Drive State", "unknown")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(3, 0)
            .withSize(2, 1)
            .getEntry();

        // Display robot coordinates - currently only Swerve odometry
        robotPoseEntry = robotStateTab
            .add("Pose X Y Angle", "0.00, 0.00, 0")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withSize(3, 1)
            .getEntry();

        // display Hood angle for troubleshooting
        hoodAngleDisplay = robotStateTab
            .add("Hood Angle (deg)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0, "max", 90))
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();

        // display turret angle for troubleshooting
        turretAngleDisplay = robotStateTab
            .add("Turret Angle (deg)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -180, "max", 180))
            .withPosition(3, 3)
            .withSize(2, 1)
            .getEntry();

        // register individual motors in ShuffleboardControl. See comments there for registration

        // ShuffleboardControl.registerOpenLoopMotor("Intake Roller", new MotorAccessor() {
        //     @Override public void setPower(double p) { intake.setPower(p); }
        //     @Override public double getPower() { return intake.getPower(); }
        //     @Override public void setSpeed(double rpm) {}
        //     @Override public double getSpeedRpm() { return 0.0; }
        //     @Override public void setPositionDegrees(double d) {}
        //     @Override public double getPositionDegrees() { return 0.0; }
        //     @Override public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) {}
        // });

        // ShuffleboardControl.registerVelocityMotor("Shooter", new MotorAccessor() {
        //     @Override public void setSpeed(double rpm) { shooter.setSpeed(rpm); }
        //     @Override public double getSpeedRpm() { return shooter.getSpeed(); }
        //     @Override public void setPower(double p) {}
        //     @Override public double getPower() { return 0.0; }
        //     @Override public void setPositionDegrees(double d) {}
        //     @Override public double getPositionDegrees() { return 0.0; }
        //     @Override public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) { shooter.applyPid(kp, ki, kd, kv, kg, mmv, mma); }
        // });

        // ShuffleboardControl.registerPositionMotor("Intake Extender", new MotorAccessor() {
        //     @Override public void setPositionDegrees(double deg) { intakeExtender.setPositionDegrees(deg); }
        //     @Override public double getPositionDegrees() { return intakeExtender.getPositionDegrees(); }
        //     @Override public void setPower(double p) {}
        //     @Override public double getPower() { return 0.0; }
        //     @Override public void setSpeed(double rpm) {}
        //     @Override public double getSpeedRpm() { return 0.0; }
        //     @Override public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) { intakeExtender.applyPid(kp, ki, kd, kv, kg, mmv, mma); }
        // });

        // ShuffleboardControl.registerPositionMotor("Hood", new MotorAccessor() {
        //     @Override public void setPositionDegrees(double deg) { hood.setPositionDegrees(deg); }
        //     @Override public double getPositionDegrees() { return hood.getPositionDegrees(); }
        //     @Override public void setPower(double p) {}
        //     @Override public double getPower() { return 0.0; }
        //     @Override public void setSpeed(double rpm) {}
        //     @Override public double getSpeedRpm() { return 0.0; }
        //     @Override public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) { hood.applyPid(kp, ki, kd, kv, kg, mmv, mma); }
        // });

        // ShuffleboardControl.registerPositionMotor("Turret", new MotorAccessor() {
        //     @Override public void setPositionDegrees(double deg) { turret.setPositionDegrees(deg); }
        //     @Override public double getPositionDegrees() { return turret.getPositionDegrees(); }
        //     @Override public void setPower(double p) {}
        //     @Override public double getPower() { return 0.0; }
        //     @Override public void setSpeed(double rpm) {}
        //     @Override public double getSpeedRpm() { return 0.0; }
        //     @Override public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) { turret.applyPid(kp, ki, kd, kv, kg, mmv, mma); }
        // });

        // ShuffleboardControl.registerOpenLoopMotor("Arm", new MotorAccessor() {
        //     @Override public void setSpeed(double rpm) {  }
        //     @Override public double getSpeedRpm() { return 0.0; }
        //     @Override public void setPower(double p) { climber.setArmPower(p); }
        //     @Override public double getPower() { return climber.getPower(); }
        //     @Override public void setPositionDegrees(double d) { }
        //     @Override public double getPositionDegrees() { return climber.getArmPosition(); }
        //     @Override public void applyPid(double kp, double ki, double kd, double kv, double kg, double mmv, double mma) { }
        // });
    }

    public Command getAutonomousCommand() {
        return AutonChooser.getSelected();

        //replace with auto chooser with pathplanner paths
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
        // //I'm the bad guy!
    }

    // helper function for changing the Robot State
    private void setRobotState(Constants.robotStates.State newState) {
        currentState = newState;
        System.out.println("Robot state changed to: " + currentState);

        if (robotStateEntry != null){
            robotStateEntry.setString(getRobotStateAsString());
        }
        else {
            System.out.println("Warning: robotStateEntry is null - Shuffleboard not Initialized yet");
        }

    }

    // convert STATE function to String for display on the Shuffleboard
    public String getRobotStateAsString() {
        return switch (currentState) {
            case FIRE_ON_THE_MOVE -> "Fire on the Move";
            case PUSH -> "Push";
            case CLIMB -> "Climb";
            case IDLE -> "Idle";
            case STOP_AND_SHOOT -> "Stop and Shoot";
            case CALIBRATION -> "Calibration";
            default -> "none";
        };
    }

    // Helper function exposing robot Pose (X, Y, Angle)
    public Pose2d getPose() {
        return drivetrain.getPose();
    }

    // Helper function exposing robot Speed (vX, vY, vOmega)
    public ChassisSpeeds getRobotSpeeds(){
        return drivetrain.getCurrentRobotRelativeSpeeds();
    }

    // Call FieldConstants and get the String value of the current field for Shuffleboard
    public String getFieldRegionString() {
        Pose2d pose = getPose();
        FieldConstants.FieldRegion region = FieldConstants.CurrentFieldState(pose);
        return FieldConstants.toString(region);
    }

    // Helper function exposing Field Region
    public FieldConstants.FieldRegion getFieldLocation() {
        return currentRegion;
    }

    // This updates robot pose and field location on Shuffleboard

    public void updateFieldAndPoseDisplay(){
        // Get robot coordinates (pose)
        Pose2d pose = getPose();

        // Get region from robot coordinates
        currentRegion = FieldConstants.CurrentFieldState(pose);

        // update Shuffleboard while we're here
        if(robotFieldEntry != null) {
            robotFieldEntry.setString(FieldConstants.toString(currentRegion));
        }
        if(robotDriveEntry != null) {
            robotDriveEntry.setString(isRobotRelative ? "Robot" : "Field");
        }
        if(robotPoseEntry != null) {
            robotPoseEntry.setString(getPoseString());
        }
        if (hoodAngleDisplay != null) {
            hoodAngleDisplay.setDouble(hood.getPositionDegrees());
        }
        if(turretAngleDisplay != null) {
            turretAngleDisplay.setDouble(turret.getPositionDegrees());
        }
    }

    // Helper function to convert pose to string for shuffleboard
    public String getPoseString() {
        Pose2d pose = getPose();
        return String.format("X: %.2f Y: %.2f Rot: %.1f", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    // Helper function to expose TurretSubsystem (for initialization of turret)
    public TurretSubsystem getTurret() {
        return turret;
    }

    // Helper function to expose Limelight function (for updating odometry)
    public LimelightSubsystem getLimelightFront(){
        return limelightFront;
    }

        // Helper function to expose Limelight function (for updating odometry)
    public LimelightSubsystem getLimelightSide(){
        return limelightSide;
    }



    public void updatePoseWithVision() {

        LimelightSubsystem ll = getLimelightFront();

        if (ll.hasValidTarget() && ll.getLatestTimestamp() > lastVisionTimestamp) {
            Pose2d visionPose = ll.getLatestPose();
            double timestamp = ll.getLatestTimestamp();

            drivetrain.addVisionMeasurement(visionPose, timestamp);

            lastVisionTimestamp = timestamp;
        }
    }

    public void updatePoseWithVisionSide() {

        LimelightSubsystem ll = getLimelightSide();

        if (ll.hasValidTarget() && ll.getLatestTimestamp() > lastVisionTimestamp) {
            Pose2d visionPose = ll.getLatestPose();
            double timestamp = ll.getLatestTimestamp();

            drivetrain.addVisionMeasurement(visionPose, timestamp);

            lastVisionTimestamp = timestamp;
        }
    }


    public void resetPoseFromLimelight() {
        LimelightSubsystem ll = getLimelightFront();

        // Limelight can take a bit to enable & stabilize (300 ms)
        Timer.delay(0.3);
        for (int i = 0; i <3; i++) {
            if (ll.hasValidTarget() && (ll.getLatestTimestamp() > 0)) {
                drivetrain.resetPose(ll.getLatestPose());
                System.out.println("Pose reset from Limelight Front at attempt " + (i+1));
                return;
            }
            Timer.delay(0.1);
        }

        ll = getLimelightSide();

        // Limelight can take a bit to enable & stabilize (300 ms)
        Timer.delay(0.3);
        for (int i = 0; i <3; i++) {
            if (ll.hasValidTarget() && (ll.getLatestTimestamp() > 0)) {
                drivetrain.resetPose(ll.getLatestPose());
                System.out.println("Pose reset from Limelight Side at attempt " + (i+1));
                return;
            }
            Timer.delay(0.1);
        }

        System.out.println("No valid Limelight pose after 3 attempts, using fallback");
        drivetrain.resetPose(FieldConstants.BLUE_LEFT_TRENCH_START);
    }

    private Command getPathfindingCommand(double x, double y, double rotateDeg){
        NetworkTableInstance nti = NetworkTableInstance.getDefault();

        var table = nti.getTable("PathPlanning");
        StructPublisher<Pose2d> drivePose = table.getStructTopic("TargetPose", Pose2d.struct).publish();

    
        Pose2d targetPose = new Pose2d(x,y, Rotation2d.fromDegrees(rotateDeg));
        drivePose.set(targetPose);

        PathConstraints constraints = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(targetPose, constraints, 0);
    }
}