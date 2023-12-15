package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode {
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
        while(!opModeIsActive()) {

            pos = webcam.CheckCamera();

            telemetry.addData("detected x", pos);
            telemetry.update();
            sleep (2000);

        }
        if(pos == Webcam.Position.Left) RunLeft(autoMethods);
        else if (pos == Webcam.Position.Right) RunRight(autoMethods);
        else RunCenter(autoMethods);
    }
    void RunLeft(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(17,0.5);
        blar.RunMotorHang(6.5,1);
        blar.StrafeByInch(13, false, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(10,false,0.4);
        blar.Turn90(true, 0.4);
        blar.StrafeByInch(3, true, 0.4);
        blar.RunMotors(17, 0.2);
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5,1);
        blar.RunMotors(-4,0.5);
        blar.StrafeByInch(20, false, 0.4);
        sleep(2000);
        motorHang.setPower(0);


    }
    void RunRight(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(25,0.4);
        blar.RunMotorHang(6.5,0.75);
        blar.StrafeByInch(10, true, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.StrafeByInch(45,false,0.4);
        motorHang.setPower(0);
        blar.Turn90(true, 0.4);
        blar.StrafeByInch(7, true, 0.4);
        blar.RunMotors(3.5, 0.2);
        blar.RunMotorHang(-6.5,0.75);
        blar.RunMotors(-4,0.5);
        blar.StrafeByInch(32, false, 0.4);
        sleep(3000);
        motorHang.setPower(0);
    }
    void RunCenter(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(26,0.4);
        blar.RunMotorHang(6.5,0.75);
        blar.StrafeByInch(4, false, 0.4);
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.RunMotors(-2,0.4);
        blar.Turn90(true, 0.4);
        blar.StrafeByInch(3, true, 0.4);
        blar.RunMotors(32, 0.4);
        blar.RunMotors(3, 0.2);
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5,0.75);
        blar.RunMotors(-4,0.5);
        blar.StrafeByInch(25, false, 0.4);
        sleep(2000);
        motorHang.setPower(0);
    }
}
