
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

        double fixedHoodDeg = ballisticConstants.FIXED_HOOD_ANGLE;

        double shooterSpeedMps = solveSpeedForFixedHood(distanceM, robotSpeeds, toTarget, targetHeightM, fixedHoodDeg);

        // make sure we returned a value in an acceptable range
        if((!Double.isNaN(shooterSpeedMps)) && (shooterSpeedMps > ballisticConstants.MIN_SHOOTER_SPEED) && (shooterSpeedMps < ballisticConstants.MAX_SHOOTER_SPEED))
        {
            double turretDeg = computeTurretAngle(robotPose, toTarget, robotSpeeds, fixedHoodDeg, shooterSpeedMps);
            return new BallisticSolution(fixedHoodDeg, turretDeg, shooterSpeedMps);
        }

        // if we can't calculate a solution for SHOOTING SPEED, fall back to the previous solver (angle bisection) and use the previous fixed exit velocity
        double hoodAngle = solveHoodAngle(distanceM, robotSpeeds, toTarget, targetHeightM);
        double turretDeg = computeTurretAngle(robotPose, toTarget, robotSpeeds, hoodAngle, ballisticConstants.SHOOTER_EXIT_VELOCITY);
        return new BallisticSolution(hoodAngle, turretDeg, ballisticConstants.SHOOTER_EXIT_VELOCITY);
    }

    private static double solveSpeedForFixedHood(
        double distanceM,
        ChassisSpeeds robotSpeeds,
        Translation2d toTarget,
        double targetHeightM,
        double fixedHoodDeg)
        {


            double thetaRad = Math.toRadians(fixedHoodDeg); // Math.sin and Math.cos take Radians, so we convert DEG to RAD
            double A = Math.cos(thetaRad); // cos(theta) and sin(theta) are constants with a fixed hood angle, so treat them as such.
            double B = Math.sin(thetaRad);
            double D = distanceM;
            double dh = targetHeightM - ballisticConstants.SHOOTER_HEIGHT; // Might want to make this Math.min(0,height)

            double robotAlong = robotNormalSpeedComponent(robotSpeeds, toTarget);

            // Solving for shooter speed is a quadratic equation in the form "a * x^2 + b * x + c = y(x)"
            // where a, b, and c are given by the values below
            // Quadratic solution is of the form (-b +- sqrt(b^2 - 4ac))/2a
            // rather than write all this out, we'll just create an a, b, and c and solve those
            // The quadratic solution thus looks a lot less messy (and less likely for the author to make an error)

            double a = dh * A * A - D * B * A;
            double b = 2 * dh * A * robotAlong - D * B * robotAlong;

            // changed to - 0.5Gravity. I think that's correct.

            double c = dh * robotAlong * robotAlong - 0.5 * ballisticConstants.GRAVITY * D * D;

            // discriminate is the part inside SQRT
            double discriminate = b * b - 4 * a * c;

            // if discriminate is negative, then there is only an imaginary solution, so return NaN (not a number)
            if (discriminate < 0) { return Double.NaN; }

            // we already have the disc to check we're not getting imaginary numbers, so let's simply things further
            double sqrtDisc = Math.sqrt(discriminate);

            // this is just the quadratic solution, but with easier labels
            double v1 = (-b + sqrtDisc) / (2 * a);
            double v2 = (-b - sqrtDisc) / (2 * a);

            // fundamental theorem of algebra
            if ((v2 > ballisticConstants.MIN_SHOOTER_SPEED) && (v2 < ballisticConstants.MAX_SHOOTER_SPEED)){ return v2; }
            if ((v1 > ballisticConstants.MIN_SHOOTER_SPEED) && (v1 < ballisticConstants.MAX_SHOOTER_SPEED)){ return v1; }

            // if there's no solution between MIN and MAX speed, return NaN
            return Double.NaN;
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
                                             ChassisSpeeds robotSpeeds, double hoodAngleDegrees,
                                             double shooterSpeed) {
        double thetaRad = Math.toRadians(hoodAngleDegrees);
        double v0 = shooterSpeed;

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

        double turretRad = launchFieldRad - robotPose.getRotation().getRadians() + Math.toRadians(180);
        double turretDeg = ((Math.toDegrees(turretRad) + 90) % 360) - 90;

        return turretDeg;
    }

    public static class BallisticSolution {
        public final double hoodAngleDegrees;
        public final double turretAngleDegrees;
        public final double shooterSpeedMps;

        public BallisticSolution(double hoodAngleDegrees, double turretAngleDegrees) {
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.turretAngleDegrees = turretAngleDegrees;
            this.shooterSpeedMps = Double.NaN;
        }

        public BallisticSolution(double hoodAngleDegrees, double turretAngleDegrees, double shooterSpeedMps){
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.turretAngleDegrees = turretAngleDegrees;
            this.shooterSpeedMps = shooterSpeedMps;
        }
    }
}
