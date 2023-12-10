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
    private boolean isLeft = true, isRight = false, isCenter = false;
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

        autoMethods = new AutoMethods(motorLeft, motorLeft2, motorRight, motorRight2, motorIntake, motorHang);
        webcam = new Webcam(hardwareMap.get(WebcamName.class, "Webcam 1"), false);
        while(!opModeIsActive()) {

            Double x = webcam.CheckCamera();
            if (x > 450 || x == 0) {
                isLeft = false;
                isRight = true;
                isCenter = false;
            } else if (x < 150) {
                isLeft = true;
                isRight = false;
                isCenter = false;
            } else {
                isLeft = true;
                isRight = false;
                isCenter = true;
            }
            telemetry.addData("detected x", x);
            telemetry.update();
            sleep (2000);

        }
        if(isLeft) RunLeft(autoMethods);
        else if (isRight) RunRight(autoMethods);
        else RunCenter(autoMethods);
    }
    void RunLeft(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(25,0.2);
        sleep(3000);
        blar.ZeroMotors();
        blar.StrafeByInch(4, false, 0.2);
        sleep(1000);
        blar.ZeroMotors();
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.Turn90(true, 0.2);
        sleep(4000);
        blar.ZeroMotors();
        blar.StrafeByInch(3, true, 0.2);
        sleep(1000);
        blar.ZeroMotors();
        blar.RunMotors(36, 0.2);
        blar.RunMotorHang(6.5,0.75);
        sleep(6000);
        blar.ZeroMotors();
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5,0.75);
        blar.RunMotors(-4,0.5);
        sleep(1000);
        blar.ZeroMotors();
        blar.StrafeByInch(24, false, 0.2);
        sleep(4000);
        blar.ZeroMotors();
        motorHang.setPower(0);


    }
    void RunRight(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(25,0.2);
        sleep(3000);
        blar.ZeroMotors();
        blar.StrafeByInch(4, false, 0.2);
        sleep(1000);
        blar.ZeroMotors();
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.Turn90(true, 0.2);
        sleep(4000);
        blar.ZeroMotors();
        blar.StrafeByInch(3, true, 0.2);
        sleep(1000);
        blar.ZeroMotors();
        blar.RunMotors(36, 0.2);
        blar.RunMotorHang(6.5,0.75);
        sleep(6000);
        blar.ZeroMotors();
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5,0.75);
        blar.RunMotors(-4,0.5);
        sleep(1000);
        blar.ZeroMotors();
        blar.StrafeByInch(24, false, 0.2);
        sleep(4000);
        blar.ZeroMotors();
        motorHang.setPower(0);
    }
    void RunCenter(AutoMethods blar) throws InterruptedException {
        blar.RunMotors(25,0.2);
        sleep(3000);
        blar.ZeroMotors();
        blar.StrafeByInch(4, false, 0.2);
        sleep(1000);
        blar.ZeroMotors();
        motorIntake.setPower(-0.4);
        sleep(1500);
        motorIntake.setPower(0);
        blar.Turn90(true, 0.2);
        sleep(4000);
        blar.ZeroMotors();
        blar.StrafeByInch(3, true, 0.2);
        sleep(1000);
        blar.ZeroMotors();
        blar.RunMotors(36, 0.2);
        blar.RunMotorHang(6.5,0.75);
        sleep(6000);
        blar.ZeroMotors();
        motorHang.setPower(0);
        blar.RunMotorHang(-6.5,0.75);
        blar.RunMotors(-4,0.5);
        sleep(1000);
        blar.ZeroMotors();
        blar.StrafeByInch(24, false, 0.2);
        sleep(4000);
        blar.ZeroMotors();
        motorHang.setPower(0);
    }
}
