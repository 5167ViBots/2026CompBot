package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightSubsystem extends SubsystemBase {

    
    private static final String TABLE_NAME = "limelight";  // default NT table
    private final NetworkTable table;

    private NetworkTableEntry tv;   // valid target (0 or 1)
    private NetworkTableEntry tx;   // horizontal offset (degrees)
    private NetworkTableEntry ty;   // vertical offset
    private NetworkTableEntry ta;   // area %
    private NetworkTableEntry tid;  // primary tag ID
    private NetworkTableEntry botpose;  // botpose_wpired (array)

    private Pose2d latestPose = new Pose2d();
    private double latestTimestamp = 0;
    private int latestTagID = -1;

    // Camera-to-robot transform (position & orientation of the camera relative to the center of the robot)
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(0.3, 0.0, 0.25),   // x forward, y left, z up (meters)
        new Rotation3d(0, 0, 0)              // camera rotation
    );

    public LimelightSubsystem(String Suffix) {
        table = NetworkTableInstance.getDefault().getTable(TABLE_NAME + "-" + Suffix);

        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");

        if (DriverStation.getAlliance().get() == Alliance.Blue)
        {
            botpose = table.getEntry("botpose_wpiblue"); // absolute coordinte pose
        }
        else {
            botpose = table.getEntry("botpose_wpired");
        }
        // botpose = table.getEntry("botpose_mt2"); // relative coordinate pose

        // Optional: set pipeline / LED mode etc. on init
        // table.getEntry("pipeline").setNumber(0);  // AprilTag pipeline
        // table.getEntry("ledMode").setNumber(0);   // default
    }

    @Override
    public void periodic() {
        // Only update if we have a valid target
        if (tv.getDouble(0) > 0.5) {
            latestTagID = (int) tid.getDouble(-1);

            // Use botpose_wpired if available (recommended for AprilTag)
            double[] poseArray = botpose.getDoubleArray(new double[6]);
            if (poseArray.length >= 6) {
                double x = poseArray[0];
                double y = poseArray[1];
                double z = poseArray[2];  // usually ignore z
                double roll = poseArray[3];
                double pitch = poseArray[4];
                double yaw = poseArray[5];

                latestPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
                latestTimestamp = Timer.getFPGATimestamp();
            }
        }
    }

    // Public getters for other code
    public boolean hasValidTarget() {
        return tv.getDouble(0) > 0.5;
    }

    public Pose2d getLatestPose() {
        return latestPose;
    }

    public double getLatestTimestamp() {
        return latestTimestamp;
    }

    public int getPrimaryTagID() {
        return latestTagID;
    }

    public double getTX() { return tx.getDouble(0.0); }
    public double getTY() { return ty.getDouble(0.0); }
    public double getTA() { return ta.getDouble(0.0); }
}