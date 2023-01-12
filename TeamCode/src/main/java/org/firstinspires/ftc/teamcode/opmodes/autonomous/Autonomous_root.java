package org.firstinspires.ftc.teamcode.opmodes.autonomous;

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

        Vision vision = new Vision("Webcam 1");
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
            arm.closeGripper();

            chassis.runToPosition(-50, -50, -50, -50);

            sleep(500);

            //arm.runToPosition(200);

            chassis.resetEncoder();

            if (vision.tagId() == LEFT) chassis.runToPosition(1100, -1400, -1400, 1100);
            else if (vision.tagId() == RIGHT) chassis.runToPosition(-1350, 1050, 1050, -1350);

            chassis.resetEncoder();

            chassis.runToPosition(-1500, -1500, -1500, -1500);

            telemetry.addLine("parked!");

            parked = true;
        }
    }
}
