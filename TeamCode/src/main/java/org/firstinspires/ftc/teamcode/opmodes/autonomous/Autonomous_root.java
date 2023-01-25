package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Chassis;
import org.firstinspires.ftc.teamcode.components.Vision;
import org.firstinspires.ftc.teamcode.components.teaminfo.InitialSide;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamColor;
import org.firstinspires.ftc.teamcode.components.teaminfo.TeamInfo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="Autonomous root :)")
public class Autonomous_root extends LinearOpMode {
    //sleeve
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

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

            if(gamepad1.b || gamepad2.b) TeamInfo.teamColor = TeamColor.RED;
            if(gamepad1.x || gamepad2.x) TeamInfo.teamColor = TeamColor.BLUE;
            if(gamepad1.dpad_right || gamepad2.dpad_right) TeamInfo.initialSide = InitialSide.RIGHT;
            if(gamepad1.dpad_left || gamepad2.dpad_left) TeamInfo.initialSide = InitialSide.LEFT;

            if(vision.tagId() > 0) telemetry.addData("Detected tag ID:", vision.tagId());
            else telemetry.addLine("Don't see tag of interest :(");

            telemetry.addData("Team color", TeamInfo.teamColor);
            telemetry.addData("Initial side", TeamInfo.initialSide);
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
            chassis.resetEncoder();

            vision.setDetector("pole");
            sleep(2500);

            for(int i=0; i<2; i++){
                //RIGHT POSITION
                chassis.runToPosition(400, -400, 400, -400);

                adjust(chassis, vision);

                chassis.resetEncoder();
                chassis.runToPosition(-220,-220,-220,-220);

                adjust(chassis, vision);

                arm.runToPosition(arm.highJunction);

                chassis.runToPosition(-450,-450,-450,-450);
                chassis.stop();

                arm.openGripper();

                //TODO: RUN TO CAM POSITION
                arm.runToPosition(0);

                telemetry.addLine("Scored one cone??");
                telemetry.update();

                vision.setDetector("cone");

                chassis.runToPosition(-220,-220,-220,-220);

                chassis.runToPosition(-1650,1250,-1650,1250);

                adjust(chassis, vision);

                chassis.resetEncoder();
                chassis.runToPosition(-600, -600, -600, -600);

                adjust(chassis, vision);

                chassis.runToPosition(-1200, -1200, -1200, -1200);

                arm.closeGripper();

                chassis.runToPosition(-50, -50, -50, -50);

                chassis.runToPosition(1000, -1000, 1000, -1000);

                chassis.resetEncoder();
            }

            if (vision.tagId() == LEFT) chassis.runToPosition(1100, -1400, -1400, 1100);
            else if (vision.tagId() == RIGHT) chassis.runToPosition(-1350, 1050, 1050, -1350);

            parked = true;
        }
    }

    void adjust(Chassis chassis, Vision vision){
        chassis.stop();
        while(Math.abs(vision.getAutonPipeline().differenceX()) > 5) {
            double power = (vision.getAutonPipeline().differenceX() < 0) ? 0.1 : -0.1;
            chassis.turn(power);
            telemetry.addData("difference", vision.getAutonPipeline().differenceX());
            telemetry.update();
        }
        chassis.stop();
    }
}
