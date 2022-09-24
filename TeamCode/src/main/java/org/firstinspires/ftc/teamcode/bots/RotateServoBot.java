package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotateServoBot extends FourWheelDriveBot {
    public CRServo parkArm = null;

    public RotateServoBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        parkArm = hwMap.crservo.get("parkArm");
        parkArm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void startExtension(int direction) {
        if (direction == 0) {
            parkArm.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (direction == 1) {
            parkArm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        parkArm.setPower(- 1);
    }

    public void stopExtension (int direction) {
        if (direction == 0) {
            parkArm.setDirection(DcMotorSimple.Direction.FORWARD);
        } else if (direction == 1) {
            parkArm.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        parkArm.setPower(0);
    }

    public void unextend () {
        parkArm.setPower(1);
    }

    public void manualExtension (boolean dpadLeft, boolean dpadRight, boolean dpadDown) {
        if (dpadLeft) {
            startExtension(0);
        }
        if (dpadRight) {
            unextend();
        }
        if (dpadDown) {
            stopExtension(0);
        }
    }
}