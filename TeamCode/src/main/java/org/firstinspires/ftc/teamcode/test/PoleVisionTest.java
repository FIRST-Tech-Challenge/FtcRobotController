package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class PoleVisionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR);
        chassis.init();

        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Arm arm = new Arm(leftLift, rightLift, gripper);
        arm.init();

        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Vision vision = new Vision(camera, telemetry, distanceSensor);
        vision.init();

        telemetry.addLine("waiting to start!");
        telemetry.update();


        while(opModeIsActive()) {
            chassis.turn(0.1);
            sleep(4000);
            chassis.turn(-0.1);
            sleep(4000);
            chassis.stop();
            vision.setPoleDetector();

            telemetry.addData("difference", vision.differenceX());
            telemetry.addData("middle", vision.middleX());
            telemetry.update();
        }
    }
}
