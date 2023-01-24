package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

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
        Vision vision = new Vision(camera, telemetry);
        vision.init();

        telemetry.addLine("waiting to start!");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            vision.searchTags();

            if(gamepad1.b || gamepad2.b) teamColor = TeamColor.red;
            if(gamepad1.x || gamepad2.x) teamColor = TeamColor.blue;
            if(gamepad1.dpad_right || gamepad2.dpad_right) initialSide = InitialSide.right;
            if(gamepad1.dpad_left || gamepad2.dpad_left) initialSide = InitialSide.left;

            if(vision.tagId() > 0) telemetry.addData("Detected tag ID:", vision.tagId());
            else telemetry.addLine("Don't see tag of interest :(");
            telemetry.update();

            sleep(20);
        }

        boolean parked = false;

        while(opModeIsActive() && !parked) {
            arm.closeGripper();
            chassis.runToPosition(-100, -100, -100, -100);

            chassis.resetEncoder();
            chassis.runToPosition(-2500, -2500, -2500, -2500);

            chassis.runToPosition(-2100, -2100, -2100, -2100);

            vision.setDetector("pole");

            for(int i=0; i<1; i++){
                //RIGHT BLUE
                chassis.runToPosition(-1600, -2400, -1600, -2400);

                //微調整(Small Adjustment)
                while(Math.abs(vision.differenceX()) > 5) {
                    double power = (vision.differenceX() < 0) ? 0.2 : -0.2;
                    chassis.turn(power);
                }
                chassis.stop();

                chassis.resetEncoder();
                chassis.runToPosition(-250,-250,-250,-250);

                chassis.stop();
                while(Math.abs(vision.differenceX()) > 5) {
                    double power = (vision.differenceX() < 0) ? 0.2 : -0.2;
                    chassis.turn(power);
                }
                chassis.stop();

                arm.runToPosition(arm.highJunction);

                chassis.runToPosition(-500,-500,-500,-500);
                chassis.stop();

                arm.openGripper();
                //RUN TO CAM POSITION
                arm.runToPosition(0);

                chassis.runToPosition(-250,-250,-250,-250);


            }

            if (vision.tagId() == LEFT) chassis.runToPosition(1100, -1400, -1400, 1100);
            else if (vision.tagId() == RIGHT) chassis.runToPosition(-1350, 1050, 1050, -1350);

            parked = true;
        }
    }
}
