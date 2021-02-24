package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="pivotLineAuton",group="chrisBot")

public class pivotLineAuton extends LinearOpMode {
    chrisBot robot = new chrisBot();
    boolean Ltripped = false, Rtripped = false;
    boolean rightFirst = false;
    int[] white = {800,800,800};
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        robot.setAllDrivePower(0.1);

        while(true) {
            if(Ltripped && Rtripped) {
                break;
            } else if (Ltripped) {
                robot.motorBackLeft.setPower(0);
                robot.motorFrontLeft.setPower(0);
                robot.motorFrontRight.setPower(0.1);
                robot.motorBackRight.setPower(0.1);
                if(white(robot.colorR)) {
                    Rtripped = true;
                }
            } else if (Rtripped) {
                robot.motorBackRight.setPower(0);
                robot.motorFrontRight.setPower(0);
                robot.motorFrontLeft.setPower(0.1);
                robot.motorBackLeft.setPower(0.1);
                if(white(robot.colorL)) {
                    Ltripped = true;
                }
            } else {
                robot.setAllDrivePower(0.1);
                if(white(robot.colorL)) {
                    Ltripped = true;
                    rightFirst = false;
                } else if(white(robot.colorR)) {
                    Rtripped = true;
                    rightFirst = true;
                }
            }
        }
        if(!rightFirst) {
            while(white(robot.colorR)) {
                robot.motorBackLeft.setPower(0);
                robot.motorFrontLeft.setPower(0);
                robot.motorFrontRight.setPower(0.1);
                robot.motorBackRight.setPower(0.1);
            }
            while(white(robot.colorL)) {
                robot.motorBackRight.setPower(0);
                robot.motorFrontRight.setPower(0);
                robot.motorFrontLeft.setPower(0.1);
                robot.motorBackLeft.setPower(0.1);
            }
        } else {
            while(white(robot.colorL)) {
                robot.motorBackRight.setPower(0);
                robot.motorFrontRight.setPower(0);
                robot.motorFrontLeft.setPower(0.1);
                robot.motorBackLeft.setPower(0.1);
            }
            while(white(robot.colorR)) {
                robot.motorBackLeft.setPower(0);
                robot.motorFrontLeft.setPower(0);
                robot.motorFrontRight.setPower(0.1);
                robot.motorBackRight.setPower(0.1);
            }
        }
        robot.setAllDrivePower(0);
    }
    public boolean white(ColorSensor c) {
        int[] color = {c.red(), c.green(), c.blue()};
        for(int i = 0; i < 3; i++) {
            if(color[i]<white[i]) {
                return false;
            }
        }
        return true;
    }
}
