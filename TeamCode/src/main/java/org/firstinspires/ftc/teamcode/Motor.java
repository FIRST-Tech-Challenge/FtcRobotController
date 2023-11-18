package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    private DcMotor dcMotor;
    private int targetPos;

    // Constructor now requires a HardwareMap to get the DcMotor instance
    Motor(HardwareMap hardwareMap, String name, int initialPosition) {
        dcMotor = hardwareMap.get(DcMotor.class, name);
        this.targetPos = initialPosition;
        dcMotor.setTargetPosition(initialPosition);
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setTargetPosition(int t) {
        dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targetPos = t;
        dcMotor.setTargetPosition(t);
    }

    void updatePosition() {
        // In the FTC SDK, the DcMotor class automatically handles moving to the target position
        // You can still print out the current position if needed
        System.err.println("updated position to " + getCurrentPosition());
    }

    int getCurrentPosition() {
        return dcMotor.getCurrentPosition();
    }

    String getName() {
        return dcMotor.getDeviceName();
    }

    // Additional methods to control the motor
    void setPower(double power) {
        dcMotor.setPower(power);
    }

    void stopMotor() {
        dcMotor.setPower(0);
    }
}
