package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

public class KinematicModel {
    private double x, y;
    private double turretX, turretY;
    private double muzzleX, muzzleY;
    private double turretHeading;
    private double leftPower, rightPower;

    public KinematicModel() {}

    public void update(double x, double y, double heading, double turretHeading, double elbowElevation, double targetTurretHeading, double targetIntakeTiltPosition, double targetLeftPower, double targetRightPower) {
        // avoiding conflicts
        // turret & ringevator interactions
//        if(targetIntakeTiltPosition < Constants.INTAKE_SERVO_TRANSIT && elbowElevation < Constants.ELBOW_LEGAL_ANGLE) {
//            double maxDistance = heading + Constants.TURRET_HEADING_OFFSET - targetTurretHeading;
//            double minDistance = heading - Constants.TURRET_HEADING_OFFSET - targetTurretHeading;
//            if(maxDistance < minDistance && maxDistance > 0)
//                turretHeading = heading + Constants.TURRET_HEADING_OFFSET;
//            else if(minDistance < maxDistance && minDistance > 0)
//                turretHeading = heading - Constants.TURRET_HEADING_OFFSET;
//        } else {
            turretHeading = targetTurretHeading;
//        }

        // drivetrain & ringevator interactions
//        if(targetIntakeTiltPosition <= Constants.INTAKE_SERVO_TRANSIT) {
//            leftPower = 0;
//            rightPower = 0;
//        } else {
            leftPower = targetLeftPower;
            rightPower = targetRightPower;
//        }

        // positions
        //turret
        turretX = x - Constants.TURRET_AXIS_OFFSET * Math.sin(heading);
        turretY = y - Constants.TURRET_AXIS_OFFSET * Math.cos(heading);

        //muzzle
        double muzzleAngleFieldRad = Math.toRadians(turretHeading)+Constants.MUZZLE_RAD_OFFSET;
        muzzleX = turretX + Constants.MUZZLE_RADIUS * Math.sin(muzzleAngleFieldRad);
        muzzleY = turretY + Constants.MUZZLE_RADIUS * Math.cos(muzzleAngleFieldRad);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getTurretX() {
        return turretX;
    }

    public void setTurretX(double turretX) {
        this.turretX = turretX;
    }

    public double getTurretY() {
        return turretY;
    }

    public void setTurretY(double turretY) {
        this.turretY = turretY;
    }

    public double getMuzzleX() {
        return muzzleX;
    }

    public void setMuzzleX(double muzzleX) {
        this.muzzleX = muzzleX;
    }

    public double getMuzzleY() {
        return muzzleY;
    }

    public void setMuzzleY(double muzzleY) {
        this.muzzleY = muzzleY;
    }

    public double getTurretHeading() {
        return turretHeading;
    }

    public void setTurretHeading(double turretHeading) {
        this.turretHeading = turretHeading;
    }

    public double getLeftPower() {
        return leftPower;
    }

    public void setLeftPower(double leftPower) {
        this.leftPower = leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public void setRightPower(double rightPower) {
        this.rightPower = rightPower;
    }
}
