package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.robotStates;

import edu.wpi.first.math.util.Units;

public final class FieldConstants {

  // // Field dimensions
  // public static final double FIELD_LENGTH_Y = Units.inchesToMeters(651.22);
  // public static final double FIELD_WIDTH_X = Units.inchesToMeters(317.69);

  // public static final double FIELD_CENTER_X = FIELD_WIDTH_X / 2.0;
  // public static final double FIELD_CENTER_Y = FIELD_LENGTH_Y / 2.0;

  // // field zones should be correct, pulled from CAD model
  // public static final double BLUE_MAX_Y = Units.inchesToMeters(158.4);
  // public static final double BLUE_DEEP_MAX_Y = Units.inchesToMeters(90.0);
  // public static final double BLUE_FRONT_MAX_Y = BLUE_MAX_Y;

  // public static final double BLUE_GATE_MIN_Y = BLUE_MAX_Y;
  // public static final double BLUE_GATE_MAX_Y = Units.inchesToMeters(205.5);
  
  // public static final double NEUTRAL_ZONE_MIN_Y = Units.inchesToMeters(181.56);
  // public static final double NEUTRAL_ZONE_MAX_Y = FIELD_LENGTH_Y - Units.inchesToMeters(181.56);

  // public static final double RED_GATE_MIN_Y = NEUTRAL_ZONE_MAX_Y;
  // public static final double RED_GATE_MAX_Y = Units.inchesToMeters(492.5);

  // public static final double RED_MIN_Y = RED_GATE_MAX_Y;
  // public static final double RED_DEEP_MIN_Y = FIELD_LENGTH_Y - BLUE_DEEP_MAX_Y;
  // public static final double RED_FRONT_MIN_Y = RED_MIN_Y;

  // public static final double LEFT_TRENCH_MIN_X = Units.inchesToMeters(0);
  // public static final double LEFT_TRENCH_MAX_X = Units.inchesToMeters(50.35);

  // public static final double LEFT_BUMP_MIN_X = LEFT_TRENCH_MAX_X + Units.inchesToMeters(12.0);
  // public static final double LEFT_BUMP_MAX_X = LEFT_BUMP_MIN_X + Units.inchesToMeters(73.00);

  // public static final double RIGHT_BUMP_MIN_X = FIELD_WIDTH_X - LEFT_BUMP_MAX_X;
  // public static final double RIGHT_BUMP_MAX_X = RIGHT_BUMP_MIN_X + Units.inchesToMeters(73.00);

  // public static final double RIGHT_TRENCH_MIN_X = FIELD_WIDTH_X - Units.inchesToMeters(50.59);
  // public static final double RIGHT_TRENCH_MAX_X = FIELD_WIDTH_X;

  // // objects in the field

  // public static final double BLUE_DEPOT_MIN_Y = Units.inchesToMeters(0.0);
  // public static final double BLUE_DEPOT_MAX_Y = BLUE_DEPOT_MIN_Y + Units.inchesToMeters(27.0);
  // public static final double BLUE_DEPOT_MIN_X = Units.inchesToMeters(61.32);
  // public static final double BLUE_DEPOT_MAX_X = BLUE_DEPOT_MIN_X + Units.inchesToMeters(42.0);

  // public static final double RED_DEPOT_MIN_Y = FIELD_LENGTH_Y - Units.inchesToMeters(27.0);
  // public static final double RED_DEPOT_MAX_Y = FIELD_LENGTH_Y;
  // public static final double RED_DEPOT_MIN_X = FIELD_CENTER_X + Units.inchesToMeters(75.93)/2;
  // public static final double RED_DEPOT_MAX_X = RED_DEPOT_MIN_X + Units.inchesToMeters(42.0);

  // public static final double BLUE_TOWER_MIN_Y = Units.inchesToMeters(0);
  // public static final double BLUE_TOWER_MAX_Y = Units.inchesToMeters(44.95);
  // public static final double BLUE_TOWER_MIN_X = Units.inchesToMeters(146.28);
  // public static final double BLUE_TOWER_MAX_X = BLUE_TOWER_MIN_X + Units.inchesToMeters(47.0);

  // public static final double RED_TOWER_MIN_Y = FIELD_LENGTH_Y - Units.inchesToMeters(44.95);
  // public static final double RED_TOWER_MAX_Y = FIELD_LENGTH_Y;
  // public static final double RED_TOWER_MIN_X = Units.inchesToMeters(123.36);
  // public static final double RED_TOWER_MAX_X = RED_TOWER_MIN_X + Units.inchesToMeters(47.0);

  // public static final double BLUE_HUB_MIN_Y = BLUE_GATE_MIN_Y;
  // public static final double BLUE_HUB_MAX_Y = BLUE_GATE_MAX_Y;
  // public static final double BLUE_HUB_MIN_X = LEFT_BUMP_MAX_X;
  // public static final double BLUE_HUB_MAX_X = RIGHT_BUMP_MAX_X;

  // public static final double RED_HUB_MIN_Y = RED_GATE_MIN_Y;
  // public static final double RED_HUB_MAX_Y = RED_GATE_MAX_Y;
  // public static final double RED_HUB_MIN_X = LEFT_BUMP_MAX_X;
  // public static final double RED_HUB_MAX_X = RIGHT_BUMP_MAX_X;

  // // target locations

  // public static final double RED_OUTPOST_CENTER_Y = FIELD_LENGTH_Y;
  // public static final double RED_OUTPOST_CENTER_X = Units.inchesToMeters(25.62);

  // public static final double BLUE_OUTPOST_CENTER_Y = Units.inchesToMeters(0.0);
  // public static final double BLUE_OUTPOST_CENTER_X = FIELD_WIDTH_X - Units.inchesToMeters(25.62);

  // public static final double RED_HUB_TARGET_Y = (RED_HUB_MAX_Y - RED_HUB_MIN_Y) / 2.0;
  // public static final double RED_HUB_TARGET_X = (RED_HUB_MAX_X - RED_HUB_MIN_X) / 2.0;

  // public static final double BLUE_HUB_TARGET_X = (BLUE_HUB_MAX_X - BLUE_HUB_MIN_X) / 2.0;
  // public static final double BLUE_HUB_TARGET_Y = (BLUE_HUB_MAX_Y - BLUE_HUB_MIN_Y) / 2.0;

  // public static final double BLUE_LEFT_TARGET_X = Units.inchesToMeters(50.0);
  // public static final double BLUE_RIGHT_TARGET_X = FIELD_WIDTH_X - BLUE_LEFT_TARGET_X;
  // public static final double BLUE_TARGET_Y = Units.inchesToMeters(50);

  // public static final double RED_LEFT_TARGET_X = FIELD_WIDTH_X - BLUE_LEFT_TARGET_X;
  // public static final double RED_RIGHT_TARGET_X = FIELD_WIDTH_X - BLUE_RIGHT_TARGET_X;
  // public static final double RED_TARGET_Y = FIELD_LENGTH_Y - BLUE_TARGET_Y;

  // public static final Translation2d BLUE_HUB_TARGET = new Translation2d(BLUE_HUB_TARGET_X, BLUE_HUB_TARGET_Y);
  // public static final Translation2d BLUE_LEFT_TARGET = new Translation2d(BLUE_LEFT_TARGET_X, BLUE_TARGET_Y);
  // public static final Translation2d BLUE_RIGHT_TARGET = new Translation2d(BLUE_RIGHT_TARGET_X, BLUE_TARGET_Y);

  // public static final Translation2d RED_HUB_TARGET = new Translation2d(RED_HUB_TARGET_X, RED_HUB_TARGET_Y);
  // public static final Translation2d RED_LEFT_TARGET = new Translation2d(RED_LEFT_TARGET_X, RED_TARGET_Y);
  // public static final Translation2d RED_RIGHT_TARGET = new Translation2d(RED_RIGHT_TARGET_X, RED_TARGET_Y);

  // Field dimensions (swapped X/Y)
  public static final double FIELD_LENGTH_X = Units.inchesToMeters(651.22);  // now X (length)
  public static final double FIELD_WIDTH_Y  = Units.inchesToMeters(317.69);  // now Y (width)

  public static final double FIELD_CENTER_X = FIELD_LENGTH_X / 2.0;
  public static final double FIELD_CENTER_Y = FIELD_WIDTH_Y / 2.0;

  // field zones (swapped and mirrored where needed)
  public static final double BLUE_MAX_X = Units.inchesToMeters(158.4);       // was Y → now X
  public static final double BLUE_DEEP_MAX_X = Units.inchesToMeters(90.0);
  public static final double BLUE_FRONT_MAX_X = BLUE_MAX_X;

  public static final double BLUE_GATE_MIN_X = BLUE_MAX_X;
  public static final double BLUE_GATE_MAX_X = Units.inchesToMeters(205.5);

  public static final double NEUTRAL_ZONE_MIN_X = Units.inchesToMeters(181.56);
  public static final double NEUTRAL_ZONE_MAX_X = FIELD_LENGTH_X - Units.inchesToMeters(181.56);

  public static final double RED_GATE_MIN_X = NEUTRAL_ZONE_MAX_X - Units.inchesToMeters(23.5);
  public static final double RED_GATE_MAX_X = Units.inchesToMeters(492.5);

  public static final double RED_MIN_X = RED_GATE_MAX_X;
  public static final double RED_DEEP_MIN_X = FIELD_LENGTH_X - BLUE_DEEP_MAX_X;
  public static final double RED_FRONT_MIN_X = RED_MIN_X;

  public static final double LEFT_TRENCH_MIN_Y = Units.inchesToMeters(0);
  public static final double LEFT_TRENCH_MAX_Y = Units.inchesToMeters(50.35);

  public static final double LEFT_BUMP_MIN_Y = LEFT_TRENCH_MAX_Y + Units.inchesToMeters(12.0);
  public static final double LEFT_BUMP_MAX_Y = LEFT_BUMP_MIN_Y + Units.inchesToMeters(73.00);

  public static final double RIGHT_BUMP_MIN_Y = FIELD_WIDTH_Y - LEFT_BUMP_MAX_Y;
  public static final double RIGHT_BUMP_MAX_Y = RIGHT_BUMP_MIN_Y + Units.inchesToMeters(73.00);

  public static final double RIGHT_TRENCH_MIN_Y = FIELD_WIDTH_Y - Units.inchesToMeters(50.59);
  public static final double RIGHT_TRENCH_MAX_Y = FIELD_WIDTH_Y;

  // objects in the field (swapped)

  public static final double BLUE_DEPOT_MIN_X = Units.inchesToMeters(0.0);
  public static final double BLUE_DEPOT_MAX_X = BLUE_DEPOT_MIN_X + Units.inchesToMeters(27.0);
  public static final double BLUE_DEPOT_MIN_Y = Units.inchesToMeters(61.32);
  public static final double BLUE_DEPOT_MAX_Y = BLUE_DEPOT_MIN_Y + Units.inchesToMeters(42.0);

  public static final double RED_DEPOT_MIN_X = FIELD_LENGTH_X - Units.inchesToMeters(27.0);
  public static final double RED_DEPOT_MAX_X = FIELD_LENGTH_X;
  public static final double RED_DEPOT_MIN_Y = FIELD_CENTER_Y + Units.inchesToMeters(75.93)/2;
  public static final double RED_DEPOT_MAX_Y = RED_DEPOT_MIN_Y + Units.inchesToMeters(42.0);

  public static final double BLUE_TOWER_MIN_X = Units.inchesToMeters(0);
  public static final double BLUE_TOWER_MAX_X = Units.inchesToMeters(44.95);
  public static final double BLUE_TOWER_MIN_Y = Units.inchesToMeters(146.28);
  public static final double BLUE_TOWER_MAX_Y = BLUE_TOWER_MIN_Y + Units.inchesToMeters(47.0);

  public static final double RED_TOWER_MIN_X = FIELD_LENGTH_X - Units.inchesToMeters(44.95);
  public static final double RED_TOWER_MAX_X = FIELD_LENGTH_X;
  public static final double RED_TOWER_MIN_Y = Units.inchesToMeters(123.36);
  public static final double RED_TOWER_MAX_Y = RED_TOWER_MIN_Y + Units.inchesToMeters(47.0);

  public static final double BLUE_HUB_MIN_X = BLUE_GATE_MIN_X;
  public static final double BLUE_HUB_MAX_X = BLUE_GATE_MAX_X;
  public static final double BLUE_HUB_MIN_Y = LEFT_BUMP_MAX_Y;
  public static final double BLUE_HUB_MAX_Y = RIGHT_BUMP_MAX_Y;

  public static final double RED_HUB_MIN_X = RED_GATE_MIN_X;
  public static final double RED_HUB_MAX_X = RED_GATE_MAX_X;
  public static final double RED_HUB_MIN_Y = LEFT_BUMP_MAX_Y;
  public static final double RED_HUB_MAX_Y = RIGHT_BUMP_MAX_Y;

  // target locations (swapped)

  public static final double RED_OUTPOST_CENTER_X = FIELD_LENGTH_X;
  public static final double RED_OUTPOST_CENTER_Y = Units.inchesToMeters(25.62);

  public static final double BLUE_OUTPOST_CENTER_X = Units.inchesToMeters(0.0);
  public static final double BLUE_OUTPOST_CENTER_Y = FIELD_WIDTH_Y - Units.inchesToMeters(25.62);

  public static final double RED_HUB_TARGET_X = (RED_HUB_MAX_X - RED_HUB_MIN_X) / 2.0;
  public static final double RED_HUB_TARGET_Y = (RED_HUB_MAX_Y - RED_HUB_MIN_Y) / 2.0;

  public static final double BLUE_HUB_TARGET_X = (BLUE_HUB_MAX_X - BLUE_HUB_MIN_X) / 2.0;
  public static final double BLUE_HUB_TARGET_Y = (BLUE_HUB_MAX_Y - BLUE_HUB_MIN_Y) / 2.0;

  public static final double BLUE_LEFT_TARGET_Y = Units.inchesToMeters(50.0);
  public static final double BLUE_RIGHT_TARGET_Y = FIELD_WIDTH_Y - BLUE_LEFT_TARGET_Y;
  public static final double BLUE_TARGET_X = Units.inchesToMeters(50);

  public static final double RED_LEFT_TARGET_Y = FIELD_WIDTH_Y - BLUE_LEFT_TARGET_Y;
  public static final double RED_RIGHT_TARGET_Y = FIELD_WIDTH_Y - BLUE_RIGHT_TARGET_Y;
  public static final double RED_TARGET_X = FIELD_LENGTH_X - BLUE_TARGET_X;

  public static final Translation2d BLUE_HUB_TARGET = new Translation2d(BLUE_HUB_TARGET_X, BLUE_HUB_TARGET_Y);
  public static final Translation2d BLUE_LEFT_TARGET = new Translation2d(BLUE_TARGET_X, BLUE_LEFT_TARGET_Y);
  public static final Translation2d BLUE_RIGHT_TARGET = new Translation2d(BLUE_TARGET_X, BLUE_RIGHT_TARGET_Y);

  public static final Translation2d RED_HUB_TARGET = new Translation2d(RED_HUB_TARGET_X, RED_HUB_TARGET_Y);
  public static final Translation2d RED_LEFT_TARGET = new Translation2d(RED_TARGET_X, RED_LEFT_TARGET_Y);
  public static final Translation2d RED_RIGHT_TARGET = new Translation2d(RED_TARGET_X, RED_RIGHT_TARGET_Y);

  public static final Pose2d BLUE_LEFT_TRENCH_START = new Pose2d(1.0, 1.0, new Rotation2d(0.0));
  public static final Pose2d RED_LEFT_TRENCH_START = new Pose2d((FIELD_LENGTH_X - 1.0), (FIELD_WIDTH_Y - 1.0), new Rotation2d(180.0));

  // Right is "DOWN" on the official map; Blue is "RIGHT" on the official map
  // 100" in "Y" direction puts us roughly in the center of the bump

  public static final Pose2d NEUTRAL_BLUE_LEFT_POSE2D   =   new Pose2d(BLUE_GATE_MAX_X + Units.inchesToMeters(50),                  Units.inchesToMeters(100), new Rotation2d(0.0));
  public static final Pose2d NEUTRAL_BLUE_RIGHT_POSE2D  =   new Pose2d(BLUE_GATE_MAX_X + Units.inchesToMeters(50),  FIELD_WIDTH_Y - Units.inchesToMeters(100), new Rotation2d(0.0));

  public static final Pose2d NEUTRAL_RED_LEFT_POSE2D    =   new Pose2d(RED_GATE_MAX_X - Units.inchesToMeters(50),                  Units.inchesToMeters(100), new Rotation2d(180.0));
  public static final Pose2d NEUTRAL_RED_RIGHT_POSE2D   =   new Pose2d(RED_GATE_MAX_X - Units.inchesToMeters(50),  FIELD_WIDTH_Y - Units.inchesToMeters(100), new Rotation2d(180.0));

  public static final Pose2d BLUE_ALLIANCE_LEFT_POSE2D  =   new Pose2d(BLUE_DEEP_MAX_X,                 Units.inchesToMeters(100), new Rotation2d(0.0));
  public static final Pose2d BLUE_ALLIANCE_RIGHT_POSE2D =   new Pose2d(BLUE_DEEP_MAX_X, FIELD_WIDTH_Y - Units.inchesToMeters(100), new Rotation2d(0.0));

  public static final Pose2d RED_ALLIANCE_LEFT_POSE2D   =   new Pose2d(RED_DEEP_MIN_X,                  Units.inchesToMeters(100), new Rotation2d(180.0));
  public static final Pose2d RED_ALLIANCE_RIGHT_POSE2D  =   new Pose2d(RED_DEEP_MIN_X,  FIELD_WIDTH_Y - Units.inchesToMeters(100), new Rotation2d(180.0));
  
  public enum FieldRegion {
    BLUE_DEEP_LEFT,
    BLUE_DEEP_RIGHT,
    BLUE_FRONT_LEFT,
    BLUE_FRONT_RIGHT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT,
    RED_DEEP_LEFT,
    RED_DEEP_RIGHT,
    RED_FRONT_LEFT,
    RED_FRONT_RIGHT,
    RED_LEFT_TRENCH,
    RED_RIGHT_TRENCH,
    RED_LEFT_BUMP,
    RED_RIGHT_BUMP,
    BLUE_LEFT_TRENCH,
    BLUE_RIGHT_TRENCH,
    BLUE_RIGHT_BUMP,
    BLUE_LEFT_BUMP,
    UNKNOWN
  }

  // NEED TO CHECK IF WE'RE ON BLUE OR RED SIDE
  // public static boolean isScoringAllowed(FieldRegion region) {
  //   switch (region) {
  //     case BLUE_DEEP_LEFT:
  //     case BLUE_DEEP_RIGHT:
  //     case BLUE_FRONT_LEFT:
  //     case BLUE_FRONT_RIGHT:
  //       return true; // Scoring allowed in alliance zone
  //     default:
  //       return false; // Scoring not allowed in neutral or opponent zones
  //   }
  // }

  // left/right is from RED perspective, with origin at 0,0 (right side blue)

  public static String toString(FieldRegion region) {
    switch (region) {
      case BLUE_DEEP_LEFT:
        return "Blue Deep Left";
      case BLUE_DEEP_RIGHT:
        return "Blue Deep Right";
      case BLUE_FRONT_LEFT:
        return "Blue Front Left";
      case BLUE_FRONT_RIGHT:
        return "Blue Front Right";
      case NEUTRAL_LEFT:
        return "Neutral Left";
      case NEUTRAL_RIGHT:
        return "Neutral Right";
      case BLUE_LEFT_TRENCH:
        return "Blue Left Trench";
      case BLUE_RIGHT_TRENCH:
        return "Blue Right Trench";
      case BLUE_LEFT_BUMP:
        return "Blue Left Bump";
      case BLUE_RIGHT_BUMP:
        return "Blue Right Bump";
      case RED_LEFT_BUMP:
        return "Red Left Bump";
      case RED_RIGHT_BUMP:
        return "Red Right Bump";
      case RED_LEFT_TRENCH:
        return "Red Left Trench";
      case RED_RIGHT_TRENCH:
        return "Red Right Trench";
      case RED_DEEP_LEFT:
        return "Red Deep Left";
      case RED_DEEP_RIGHT:
        return "Red Deep Right";
      case RED_FRONT_LEFT:
        return "Red Front Left";
      case RED_FRONT_RIGHT:
        return "Red Front Right";
      default:
        return "Unknown Field Region";
    }
  }


  // swapped all X's & Y's in here too.
  public static FieldRegion CurrentFieldState(Pose2d robotPose2D){
    double y = robotPose2D.getY();
 
    double x = robotPose2D.getX();

    if(x < 0 || x > FIELD_LENGTH_X || y < 0 || y > FIELD_WIDTH_Y){
      return FieldRegion.UNKNOWN; // Out of bounds
    }

    if(y < FIELD_CENTER_Y){
      // Left side of the field
      if(x < BLUE_DEEP_MAX_X){
        return FieldRegion.BLUE_DEEP_LEFT;
      } else if (x < BLUE_FRONT_MAX_X){
        return FieldRegion.BLUE_FRONT_LEFT;
      } else if (x < BLUE_GATE_MAX_X){
          if (y < LEFT_TRENCH_MAX_Y){
            return FieldRegion.BLUE_LEFT_TRENCH;
          } else if (y < LEFT_BUMP_MAX_Y){
            return FieldRegion.BLUE_LEFT_BUMP;
          }
      } else if (x > RED_GATE_MIN_X && x < RED_GATE_MAX_X){
        if (y < LEFT_TRENCH_MAX_Y){
          return FieldRegion.RED_LEFT_TRENCH;
        } else if (y < LEFT_BUMP_MAX_Y){
          return FieldRegion.RED_LEFT_BUMP;
        }
       } else if (x > RED_FRONT_MIN_X && x < RED_DEEP_MIN_X){
        return FieldRegion.RED_FRONT_LEFT;
       } else if (x > RED_DEEP_MIN_X){
        return FieldRegion.RED_DEEP_LEFT;
      } else {
        return FieldRegion.NEUTRAL_LEFT;
      }
    } else {
      // Right side of the field
      if(x < BLUE_DEEP_MAX_X){
        return FieldRegion.BLUE_DEEP_RIGHT;
      } else if (x < BLUE_FRONT_MAX_X){
        return FieldRegion.BLUE_FRONT_RIGHT;
      } else if (x < BLUE_GATE_MAX_X){
          if (y < RIGHT_BUMP_MAX_Y){
            return FieldRegion.BLUE_RIGHT_BUMP;
          } else if (y < RIGHT_TRENCH_MAX_Y){
            return FieldRegion.BLUE_RIGHT_TRENCH;
          }
      } else if (x > RED_GATE_MIN_X && x < RED_GATE_MAX_X){
        if (y > RIGHT_TRENCH_MIN_Y){
          return FieldRegion.RED_RIGHT_TRENCH;
        } else if (y > RIGHT_BUMP_MIN_Y){
          return FieldRegion.RED_RIGHT_BUMP;
        }
       } else if (x > RED_FRONT_MIN_X && x < RED_DEEP_MIN_X){
        return FieldRegion.RED_FRONT_RIGHT;
       } else if (x > RED_DEEP_MIN_X){
        return FieldRegion.RED_DEEP_RIGHT;
      } else {
        return FieldRegion.NEUTRAL_RIGHT;
      }
    }
    return FieldRegion.UNKNOWN; // Default case if none of the conditions are met (this should never happen)
  }
}