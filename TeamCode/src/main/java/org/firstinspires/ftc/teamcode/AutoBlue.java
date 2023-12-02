package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Blue", group = "Autonomous")
public class AutoBlue extends LinearOpMode {

    private Servo leftGrip;
    private Servo rightGrip;
    private DcMotor armRotate;
    private DcMotor armBrace;
    private DcMotor armExt;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {

        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                backRightMotor.setPower(-0.3);
                backLeftMotor.setPower(1);
                frontRightMotor.setPower(-0.3);
                frontLeftMotor.setPower(1);
                sleep(1000);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
            }
        }
    }
}
