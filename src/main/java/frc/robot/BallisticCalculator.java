package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.ballisticConstants;
import frc.robot.Constants.HoodConstants;

public final class BallisticCalculator {

    public static BallisticSolution getAngles(
            Pose2d robotPose,
            Translation2d targetPosition,
            ChassisSpeeds robotSpeeds,      // field-relative vx, vy (m/s)
            double targetHeightM) {

        Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
        double distanceM = toTarget.getNorm();

        if (distanceM < 0.1) {
            return new BallisticSolution(45.0, 0.0); // fallback for very close range
        }

        double hoodAngle = solveHoodAngle(distanceM, robotSpeeds, toTarget, targetHeightM);

        double turretDeg = computeTurretAngle(robotPose, toTarget, robotSpeeds, hoodAngle);

        return new BallisticSolution(hoodAngle, turretDeg);
    }

    private static double solveHoodAngle(double distanceM, ChassisSpeeds robotSpeeds, Translation2d toTarget, double targetHeightM) {

        double lowAscending = HoodConstants.LOWER_LIMIT;
        double highAscending = 45.0; // cap on the low ascending trajectory

        double lowDescending = 40.0;
        double highDescending = HoodConstants.UPPER_LIMIT;

        double bestAscending = Double.NaN;
        double bestDescending = Double.NaN;

        // run the first time for a low-angle solution
        for (int i = 0; i < ballisticConstants.MAX_ATTEMPTS; i++) {
            double mid = (lowAscending + highAscending) / 2.0;
            double error = projectileError(mid, distanceM, robotSpeeds, toTarget, targetHeightM);

            if (Math.abs(error) <= ballisticConstants.HOOD_HEIGHT_ERROR_TOLERANCE) {
                bestAscending = mid;
                break;
            } else if (error > 0) {
                highAscending = mid;   // ball too high → lower angle
            } else {
                lowAscending = mid;    // ball too low → raise angle
            }
        }

        if (Double.isNaN(bestAscending)){
            bestAscending = (lowAscending + highAscending) / 2.0; // best guess if no solution found
        }

        // run the second time for a high-angle solution
        for (int j = 0; j < ballisticConstants.MAX_ATTEMPTS; j++) {
            double mid = (lowDescending + highDescending) / 2.0;
            double error = projectileError(mid, distanceM, robotSpeeds, toTarget, targetHeightM);

            if (Math.abs(error) <= ballisticConstants.HOOD_HEIGHT_ERROR_TOLERANCE) {
                bestDescending = mid;
                break;
            } else if (error > 0) {
                highDescending = mid;   // ball too high → lower angle
            } else {
                lowDescending = mid;    // ball too low → raise angle
            }
        }
        if (Double.isNaN(bestDescending)){
            bestDescending = (lowDescending + highDescending) / 2.0; // best guess if no solution found
        }
        if(!Double.isNaN(bestDescending) && bestDescending > - 35.0) { // arbitrary cap to avoid extreme angles, can be tuned based on testing
            //  check velocity at target for both solutions and determine whether ball is ascending or descending
            double vyatTargetDesc = calculateVyAtTarget(bestDescending, distanceM, robotSpeeds, toTarget);
            if (vyatTargetDesc < 0) {
                return bestDescending;
            }
        }
        return bestAscending; // fallback if the Descending solution doesn't work out.
    }

    private static double calculateVyAtTarget(double angleDegrees, double distanceM, ChassisSpeeds robotSpeeds, Translation2d toTarget) {
        double thetaRad = Math.toRadians(angleDegrees);
        double v0 = ballisticConstants.SHOOTER_EXIT_VELOCITY;

        double robotAlong = robotNormalSpeedComponent(robotSpeeds, toTarget);
        double vx = v0 * Math.cos(thetaRad) + robotAlong;

        if (vx <= 0.01) {
            return 0.0; // unreachable
        }

        double flightTime = distanceM / vx;
        double vy = v0 * Math.sin(thetaRad);
        return (vy - ballisticConstants.GRAVITY * flightTime);
    }

    private static double projectileError(double angleDegrees, double distanceM, ChassisSpeeds robotSpeeds, Translation2d toTarget, double targetHeightM) {
        double thetaRad = Math.toRadians(angleDegrees);
        double v0 = ballisticConstants.SHOOTER_EXIT_VELOCITY;

        double robotAlong = robotNormalSpeedComponent(robotSpeeds, toTarget);
        double vx = v0 * Math.cos(thetaRad) + robotAlong;

        if (vx <= 0.01) {
            return 1000.0; // unreachable
        }

        double flightTime = distanceM / vx;
        double vy = v0 * Math.sin(thetaRad);
        double yAtTarget = ballisticConstants.SHOOTER_HEIGHT + vy * flightTime -
                           0.5 * ballisticConstants.GRAVITY * flightTime * flightTime;

        return yAtTarget - targetHeightM;
    }

    private static double robotNormalSpeedComponent(ChassisSpeeds speeds, Translation2d toTarget) {
        double dist = toTarget.getNorm();
        if (dist < 0.1) return 0.0;

        Translation2d unitToTarget = toTarget.div(dist);
        return speeds.vxMetersPerSecond * unitToTarget.getX() +
               speeds.vyMetersPerSecond * unitToTarget.getY();
    }

    private static double computeTurretAngle(Pose2d robotPose, Translation2d toTarget,
                                             ChassisSpeeds robotSpeeds, double hoodAngleDegrees) {
        double thetaRad = Math.toRadians(hoodAngleDegrees);
        double v0 = ballisticConstants.SHOOTER_EXIT_VELOCITY;

        double robotAlong = robotNormalSpeedComponent(robotSpeeds, toTarget);
        double vx = v0 * Math.cos(thetaRad) + robotAlong;

        if (vx <= 0.01) {
            return 0.0; // fallback
        }

        double flightTime = toTarget.getNorm() / vx;

        double reqVx = toTarget.getX() / flightTime;
        double reqVy = toTarget.getY() / flightTime;

        double muzzleVx = reqVx - robotSpeeds.vxMetersPerSecond;
        double muzzleVy = reqVy - robotSpeeds.vyMetersPerSecond;

        double launchFieldRad = Math.atan2(muzzleVy, muzzleVx);

        double turretRad = launchFieldRad - robotPose.getRotation().getRadians();
        double turretDeg = ((Math.toDegrees(turretRad) + 180) % 360) - 180;

        return turretDeg;
    }

    public static class BallisticSolution {
        public final double hoodAngleDegrees;
        public final double turretAngleDegrees;

        public BallisticSolution(double hoodAngleDegrees, double turretAngleDegrees) {
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.turretAngleDegrees = turretAngleDegrees;
        }
    }
}