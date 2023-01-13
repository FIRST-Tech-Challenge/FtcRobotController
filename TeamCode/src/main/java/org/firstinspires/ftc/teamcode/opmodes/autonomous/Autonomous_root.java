package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.vision.SleeveDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Autonomous root :)")
public class Autonomous_root extends LinearOpMode {
    //sleeve
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    //Team information
    private enum TeamColor {red, blue}
    private enum InitialSide {right, left}

    TeamColor teamColor;
    InitialSide initialSide;

    @Override
    public void runOpMode() {
        // init chassis
        DcMotor motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        DcMotor motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        DcMotor motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        DcMotor motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        Chassis chassis = new Chassis(motorFL, motorFR, motorBL, motorBR);
        chassis.init();

        // init arms
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightArm");
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Arm arm = new Arm(leftLift, rightLift, gripper);
        arm.init();
        arm.armTarget = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Vision vision = new Vision(camera);
        vision.init();

        telemetry.addLine("waiting to start!");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            vision.searchTags();

            if(gamepad1.b || gamepad2.b) teamColor = TeamColor.red;
            if(gamepad1.x || gamepad2.x) teamColor = TeamColor.blue;
            if(gamepad1.dpad_right || gamepad2.dpad_right) initialSide = InitialSide.right;
            if(gamepad1.dpad_left || gamepad2.dpad_left) initialSide = InitialSide.left;

            if(vision.tagId() > 0) telemetry.addLine(String.format("Detected tag ID: %d", vision.tagId()));
            else telemetry.addLine("Don't see tag of interest :(");
            telemetry.update();

            sleep(20);
        }

        boolean parked = false;

        while(opModeIsActive() && !parked) {
            //RIGHT BLUE

            arm.closeGripper();
            arm.runToPosition(arm.highJunction);

            chassis.resetEncoder();
            chassis.runToPosition(-2000, -2000, -2000, -2000);

            for(int i=0; i<1; i++){
                //RIGHT BLUE
                chassis.runToPosition(-1700, -2300, -1700, -2300);

                //微調整

                arm.openGripper();
                arm.runToPosition(0);

                //RIGHT BLUE
                chassis.runToPosition(-2300, -1700, -2300, -1700);
            }

            if (vision.tagId() == LEFT) chassis.runToPosition(1100, -1400, -1400, 1100);
            else if (vision.tagId() == RIGHT) chassis.runToPosition(-1350, 1050, 1050, -1350);

            telemetry.addLine("parked!");

            parked = true;
        }
    }
}
