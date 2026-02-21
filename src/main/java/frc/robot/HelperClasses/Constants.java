package frc.robot.HelperClasses;
// import java.util.ArrayList;
// import java.util.List;

//import frc.robot.CommandSwerveDrivetrain;


//import com.revrobotics.Rev2mDistanceSensor.Port;


public class Constants {
    public static final String RioCanBus = "rio";
    public static final String CanivoreCanBus = "Canivore";


    

    public static class ShooterSubsystemConstants {
        public static final int ShooterMotor1ID = 11;
        public static final String ShooterMotor1CAN = RioCanBus;
        public static final int ShooterMotor2ID = 12;
        public static final String ShooterMotor2CAN = RioCanBus;
        public static final double kP = 2.4;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double shooterspeed = 0.5;

        //public static final Port distanceSensorPort = Port.kMXP;
    }

    public static class ControllerPorts  {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

}




