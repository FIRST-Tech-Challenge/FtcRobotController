package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;

public class HeadingController {
    public enum Type {
        GYRO, CAMERA
    };

    private Type fType;

    private IMU gyro;
    // private Camera camera;

    private double target = 0;
    private double kp = 0.02;

    private boolean enabled = false;
    private boolean findClosestTarget;

    private Button enabledToggle;
    private Button north;
    private Button east;
    private Button west;
    private Button south;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    PIDController controller;


    public HeadingController(ExtendedGamepad gamepad1) {
        north = gamepad1.dpad_up;
        east = gamepad1.dpad_right;
        west = gamepad1.dpad_left;
        south = gamepad1.dpad_down;

        enabledToggle = gamepad1.right_stick_button;

        controller = new PIDController(kP, kI, kD);
    }

    public HeadingController(ExtendedGamepad gamepad, IMU gyro) {
        this(gamepad);
        this.gyro = gyro;
        fType = Type.GYRO;
        kP = 0.02;
        controller.setP(kP);
    }

//     public HeadingController(ExtendedGamepad gamepad, Camera camera) {
//         this(gamepad);
//         this.camera = camera;
//         fType = Type.CAMERA;
//         kP = 0.6;
//        controller.setP(kP);
//     }

    public void update() {
        updateState();
        if (fType == Type.GYRO)
            updateGyroTarget();
    }

    public double calculateTurn() {
        double curValue;
        if (fType == Type.CAMERA) {
            // curValue = camera.getObject_x();
            curValue = 0;
        } else {
            if (findClosestTarget) {
                target = gyro.findClosestOrientationTarget();
                findClosestTarget = false;
            }
            curValue = gyro.getValue();
        }

        return controller.calculate(curValue);
//        return (target - curValue) * kp;
    }

    public boolean isEnabled() {
        return enabled;
    }

    private void updateGyroTarget() {
        double gyroValue = gyro.getValue();
        double dist, minDist;
        int minDistIdx, maxIdx;
        double targetOrient = target;
        boolean isBumped = false;
        if (north.isBumped()) {
            targetOrient = 0;
            isBumped = true;
        } else if (east.isBumped()) {
            targetOrient = 90;
            isBumped = true;
        } else if (west.isBumped()) {
            targetOrient = -90;
            isBumped = true;
        } else if (south.isBumped()) {
            targetOrient = 180;
            isBumped = true;
        }

        minDistIdx = 0;
        if (isBumped) {
            minDist = Math.abs(targetOrient - gyroValue);
            maxIdx = (int) Math.ceil(Math.abs(gyroValue) / 360);
            for (int i = -maxIdx; i <= maxIdx; i++) {
                dist = Math.abs(i * 360 + targetOrient - gyroValue);
                if (dist < minDist) {
                    minDistIdx = i;
                    minDist = dist;
                }
            }
        }

        target = minDistIdx * 360 + targetOrient;
        controller.setSetPoint(target);
    }

    private void updateState() {
        if (enabledToggle.isBumped()) {
            enabled = !enabled;
            findClosestTarget = enabled ? true : findClosestTarget;
        }
    }
}