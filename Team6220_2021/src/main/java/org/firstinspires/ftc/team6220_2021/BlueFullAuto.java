package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous(name = "BlueFullAuto", group = "Autonomous")
public class BlueFullAuto extends MasterAutonomous {

    int Detection = 3;
    int ArmPosition;
    int DriveAdjust;
    double ServoPosition;
    @Override
    public void runOpMode() {
        Initialize();

        servoGrabber.setPosition(0.0);
        pauseMillis(500);
        servoArm.setPosition(0.81);
        waitForStart();
        if (Detection == 1) {
            ArmPosition = -220;
            ServoPosition = 0.6;
            DriveAdjust = 23;
        } else if (Detection == 2) {
            ArmPosition = -470;
            ServoPosition = 0.8;
            DriveAdjust = 21;
        } else if (Detection == 3) {
            ArmPosition = -720;
            ServoPosition = 1;
            DriveAdjust = 25;
        }
        forward(10, 0.3);
        turnAngle(90);
        forward(15, 0.5);
        stopBase();
        blueDuck();
        pauseMillis(2000);
        forward(-5, 0.5);
        turnAngle(-125);
        servoArm.setPosition(ServoPosition);
        motorArm.setTargetPosition(ArmPosition);
        motorArm.setPower(0.9);
        pauseMillis(500);
        forward(DriveAdjust, 0.5);
        stopBase();
        servoGrabber.setPosition(0.7);
        pauseMillis(750);
        forward(-20, 0.5);
        turnAngle(130);
        stopBase();
        servoGrabber.setPosition(0.34);
        pauseMillis(500);
        servoArm.setPosition(0.01);
        motorArm.setTargetPosition(-220);
        motorArm.setPower(0.9);
        pauseMillis(500);
        forward(-84, 0.8);
        motorArm.setTargetPosition(10);
        motorArm.setPower(0.9);
        pauseMillis(500);
    }
}