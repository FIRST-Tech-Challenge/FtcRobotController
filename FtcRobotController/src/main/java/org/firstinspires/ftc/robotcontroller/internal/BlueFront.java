package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Front", group="16757 Auto")
public class BlueFront extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDriveFront = null;
    private DcMotorEx rightDriveFront = null;
    private DcMotorEx leftDriveBack = null;
    private DcMotorEx rightDriveBack = null;

    private Servo claw0 = null;
    private Servo claw1 = null;

    private Servo pixel = null;

    private ColorSensor color = null;

    @Override
    public void runOpMode() {
        leftDriveFront  = hardwareMap.get(DcMotorEx.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotorEx.class, "right_drive_front");
        leftDriveBack  = hardwareMap.get(DcMotorEx.class, "left_drive_back");
        rightDriveBack = hardwareMap.get(DcMotorEx.class, "right_drive_back");

        claw0 = hardwareMap.get(Servo.class, "claw0");
        claw1 = hardwareMap.get(Servo.class, "claw1");

        pixel = hardwareMap.get(Servo.class, "pixel");

        color = hardwareMap.get(ColorSensor.class, "color");

        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        //claw0.setPosition(0.67f);
        //claw1.setPosition(0.0f);

        pixel.setPosition(1.0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if(runtime.time() < 0.8) {
                drive(0, -1, 0);
            } else if (runtime.time() < 7.8) {
                if(color.blue() >= color.red() && color.blue() >= color.green()){
                    drive(0, 0, 0);
                    pixel.setPosition(0.0);
                } else {
                    drive(0, 0, 0.25f);
                }
            } /*else if (runtime.time() < 12) {
                drive(0, -1, 0);
            } else if (runtime.time() < 14) {
                drive(1, 0, 0);
            } else if (runtime.time() < 16) {
                drive(0, -1, 0);
            }*/ else {
                drive(0, 0, 0);
            }
        }
    }

    void drive(float x1, float y1, float x2){
        double fl = 0.0;
        double fr = 0.0;
        double bl = 0.0;
        double br = 0.0;

        fl += y1;
        fr += y1;
        bl += y1;
        br += y1;

        fl -= x1;
        fr += x1;
        bl += x1;
        br -= x1;

        fl -= x2;
        fr += x2;
        bl -= x2;
        br += x2;

        leftDriveFront.setVelocity(fl * 1000);
        rightDriveFront.setVelocity(fr * 1000);
        leftDriveBack.setVelocity(bl * 1000);
        rightDriveBack.setVelocity(br * 1000);
    }
}