package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.BoundaryConstants;
import java.util.ArrayList;
import java.util.List;

public class BoundaryManager {

    public enum BoundaryType {
        PERMITTED,   // robot allowed inside
        FORBIDDEN    // robot NOT allowed inside — stop before entering
    }

    private static class RectangularBoundary {
        BoundaryType type;
        double minX, maxX, minY, maxY;           // true geometry
        double hardMinX, hardMaxX, hardMinY, hardMaxY;  // expanded by robot margin
        double slowMinX, slowMaxX, slowMinY, slowMaxY;  // expanded by slowdown margin

        RectangularBoundary(BoundaryType type, double minX, double maxX, double minY, double maxY) {
            this.type = type;
            this.minX = minX;
            this.maxX = maxX;
            this.minY = minY;
            this.maxY = maxY;

            // Precompute expanded bounds
            this.hardMinX = minX - BoundaryConstants.ROBOT_MARGIN;
            this.hardMaxX = maxX + BoundaryConstants.ROBOT_MARGIN;
            this.hardMinY = minY - BoundaryConstants.ROBOT_MARGIN;
            this.hardMaxY = maxY + BoundaryConstants.ROBOT_MARGIN;

            this.slowMinX = minX - BoundaryConstants.SLOWDOWN_MARGIN;
            this.slowMaxX = maxX + BoundaryConstants.SLOWDOWN_MARGIN;
            this.slowMinY = minY - BoundaryConstants.SLOWDOWN_MARGIN;
            this.slowMaxY = maxY + BoundaryConstants.SLOWDOWN_MARGIN;
        }
    }

    private final List<RectangularBoundary> boundaries = new ArrayList<>();

    public BoundaryManager() {
        // Entire field is permitted
        addPermittedArea(0.0, FieldConstants.FIELD_LENGTH_X, 0.0, FieldConstants.FIELD_WIDTH_Y);

        // Forbidden zones (true geometry only)
        addForbiddenArea(FieldConstants.BLUE_DEPOT_MIN_X, FieldConstants.BLUE_DEPOT_MAX_X,
                         FieldConstants.BLUE_DEPOT_MIN_Y, FieldConstants.BLUE_DEPOT_MAX_Y);

        addForbiddenArea(FieldConstants.BLUE_TOWER_MIN_X, FieldConstants.BLUE_TOWER_MAX_X,
                         FieldConstants.BLUE_TOWER_MIN_Y, FieldConstants.BLUE_TOWER_MAX_Y);

        addForbiddenArea(FieldConstants.BLUE_HUB_MIN_X, FieldConstants.BLUE_HUB_MAX_X,
                         FieldConstants.BLUE_HUB_MIN_Y, FieldConstants.BLUE_HUB_MAX_Y);

        addForbiddenArea(FieldConstants.BLUE_GATE_MIN_X, FieldConstants.BLUE_GATE_MAX_X,
                         FieldConstants.LEFT_TRENCH_MAX_Y, FieldConstants.LEFT_BUMP_MIN_Y);

        addForbiddenArea(FieldConstants.BLUE_GATE_MIN_X, FieldConstants.BLUE_GATE_MAX_X,
                         FieldConstants.RIGHT_TRENCH_MAX_Y, FieldConstants.RIGHT_BUMP_MIN_Y);

        addForbiddenArea(FieldConstants.RED_DEPOT_MIN_X, FieldConstants.RED_DEPOT_MAX_X,
                         FieldConstants.RED_DEPOT_MIN_Y, FieldConstants.RED_DEPOT_MAX_Y);

        addForbiddenArea(FieldConstants.RED_TOWER_MIN_X, FieldConstants.RED_TOWER_MAX_X,
                         FieldConstants.RED_TOWER_MIN_Y, FieldConstants.RED_TOWER_MAX_Y);

        addForbiddenArea(FieldConstants.RED_HUB_MIN_X, FieldConstants.RED_HUB_MAX_X,
                         FieldConstants.RED_HUB_MIN_Y, FieldConstants.RED_HUB_MAX_Y);

        addForbiddenArea(FieldConstants.RED_GATE_MIN_X, FieldConstants.RED_GATE_MAX_X,
                         FieldConstants.LEFT_TRENCH_MAX_Y, FieldConstants.LEFT_BUMP_MIN_Y);

        addForbiddenArea(FieldConstants.RED_GATE_MIN_X, FieldConstants.RED_GATE_MAX_X,
                         FieldConstants.RIGHT_TRENCH_MAX_Y, FieldConstants.RIGHT_BUMP_MIN_Y);
    }

    public void addPermittedArea(double minX, double maxX, double minY, double maxY) {
        boundaries.add(new RectangularBoundary(BoundaryType.PERMITTED, minX, maxX, minY, maxY));
    }

    public void addForbiddenArea(double minX, double maxX, double minY, double maxY) {
        boundaries.add(new RectangularBoundary(BoundaryType.FORBIDDEN, minX, maxX, minY, maxY));
    }

    public ChassisSpeeds constrainSpeeds(Pose2d currentPose, ChassisSpeeds requested) {
        double vx = requested.vxMetersPerSecond;
        double vy = requested.vyMetersPerSecond;
        double omega = requested.omegaRadiansPerSecond;

        Translation2d pos = currentPose.getTranslation();

        // Predict next position (20 ms ahead)
        double dt = 0.020;  // robot loop time in seconds
        double nextX = pos.getX() + vx * dt;
        double nextY = pos.getY() + vy * dt;

        double slowdown = 1.0;

        boolean wouldEnterForbidden = false;

        for (RectangularBoundary b : boundaries) {
            if (b.type == BoundaryType.FORBIDDEN) {
                // Check if NEXT position would be inside forbidden
                if (nextX > b.hardMinX && nextX < b.hardMaxX &&
                    nextY > b.hardMinY && nextY < b.hardMaxY) {
                    wouldEnterForbidden = true;
                    // Clamp to prevent entry
                    vx = 0;
                    vy = 0;
                    break;
                }

                // Also check if CURRENT position is inside → zero velocity
                if (pos.getX() > b.hardMinX && pos.getX() < b.hardMaxX &&
                    pos.getY() > b.hardMinY && pos.getY() < b.hardMaxY) {
                    vx = 0;
                    vy = 0;
                    break;
                }
            }
            else { // PERMITTED
                // Predict if next position would be outside permitted
                if (nextX < b.hardMinX && vx < 0) vx = 0;
                if (nextX > b.hardMaxX && vx > 0) vx = 0;
                if (nextY < b.hardMinY && vy < 0) vy = 0;
                if (nextY > b.hardMaxY && vy > 0) vy = 0;

                // If current position is outside → allow slow return
                if (pos.getX() < b.hardMinX && vx < 0) vx = Math.max(vx, 0.4);  // slow crawl right
                if (pos.getX() > b.hardMaxX && vx > 0) vx = Math.min(vx, -0.4); // slow crawl left
                if (pos.getY() < b.hardMinY && vy < 0) vy = Math.max(vy, 0.4);
                if (pos.getY() > b.hardMaxY && vy > 0) vy = Math.min(vy, -0.4);

                // Slowdown near boundary
                double slowDistLeft   = pos.getX() - b.slowMinX;
                double slowDistRight  = b.slowMaxX - pos.getX();
                double slowDistBottom = pos.getY() - b.slowMinY;
                double slowDistTop    = b.slowMaxY - pos.getY();

                slowdown = Math.min(slowdown, Math.max(0.1, slowDistLeft   / BoundaryConstants.SLOWDOWN_MARGIN));
                slowdown = Math.min(slowdown, Math.max(0.1, slowDistRight  / BoundaryConstants.SLOWDOWN_MARGIN));
                slowdown = Math.min(slowdown, Math.max(0.1, slowDistBottom / BoundaryConstants.SLOWDOWN_MARGIN));
                slowdown = Math.min(slowdown, Math.max(0.1, slowDistTop    / BoundaryConstants.SLOWDOWN_MARGIN));
            }
        }

        // Apply slowdown only if not in forbidden (forbidden already zeroed)
        if (!wouldEnterForbidden) {
            vx *= slowdown;
            vy *= slowdown;
        }

        // Debug print
        if ((vx != requested.vxMetersPerSecond) || (vy != requested.vyMetersPerSecond)) {
            System.out.println("Pose x = " + pos.getX() + " y = " + pos.getY());
            System.out.println("Constrained: vx=" + vx + " vy=" + vy + " (original: " + requested.vxMetersPerSecond + ", " + requested.vyMetersPerSecond + ")");
        }

        return new ChassisSpeeds(vx, vy, omega);
    }
}