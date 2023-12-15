package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "BlueRight")
public class BlueRight extends LinearOpMode {
    Webcam webcam;
    private AutoMethods autoMethods;

    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorIntake, motorHang;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("front_Left");
        motorRight = hardwareMap.dcMotor.get("front_Right");
        motorLeft2 = hardwareMap.dcMotor.get("back_Left");
        motorRight2 = hardwareMap.dcMotor.get("back_Right");
        motorIntake = hardwareMap.dcMotor.get("Intake");
        motorHang = hardwareMap.dcMotor.get("Hanger");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Webcam.Position pos= Webcam.Position.Left;

        autoMethods = new AutoMethods(motorLeft, motorLeft2, motorRight, motorRight2, motorIntake, motorHang);
        webcam = new Webcam(hardwareMap.get(WebcamName.class, "Webcam 1"), false);
        while(!opModeIsActive()){

            pos = webcam.CheckCamera();

            telemetry.addData("detected x", pos);
            telemetry.update();
            sleep (2000);

        }
        waitForStart();
        if(pos == Webcam.Position.Left) RunLeft(autoMethods);
        else if (pos == Webcam.Position.Right) RunRight(autoMethods);
        else RunCenter(autoMethods);
    }
    void RunLeft(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(20,0.2);
        blar.Turn90(true, 0.2);
        blar.StrafeByInch(5,true,0.2);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(24, true, 0.2);
        blar.RunMotorHang(6.5,0.75);
        blar.RunMotors(83,0.2);
        blar.StrafeByInch(25, false, 0.2);
        motorHang.setPower(0);
        blar.RunMotors(5, 0.2);
        blar.RunMotorHang(-6.5,1);
        blar.RunMotors(-4,0.5);
        sleep(4000);
        motorHang.setPower(0);


    }
    void RunRight(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(17,0.5);
        blar.StrafeByInch(9, true, 0.2);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(9, false, 0.2);
        blar.RunMotors(32,0.2);
        blar.Turn90(true, 0.2);
        blar.RunMotorHang(6.5,1);
        blar.RunMotors(83, 0.5);
        blar.StrafeByInch(12, false, 0.2);
        blar.RunMotors(4,0.2);
        blar.RunMotors(-4,0.2);
        blar.RunMotorHang(-6.5,0.75);
        sleep(5000);
        motorHang.setPower(0);
    }
    void RunCenter(AutoMethods blar) throws InterruptedException {
        sleep(5000);
        blar.RunMotors(25,0.2);
        blar.StrafeByInch(4, false, 0.2);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.RunMotorHang(6.5,0.2);
        blar.StrafeByInch(72, false, 0.2);
        blar.Turn90(true, 0.2);
        blar.StrafeByInch(4, true, 0.2);
        blar.RunMotors(12, 0.2);
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5,1);
        blar.RunMotors(-4,0.5);
        blar.ZeroMotors();
        sleep(4000);
        motorHang.setPower(0);
    }

}
