package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;
import org.firstinspires.ftc.teamcode.opencv.ShippingElementRecognizer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Non-Roadrunner Cycle Auto (Red)", group = "Autonomous")
public class CycleRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumChassis chassis = new MecanumChassis();
    private Lift lift = new Lift();
    private Hopper hopper = new Hopper();
    private Intake intake = new Intake();
    OpenCvWebcam webcam;

    public void runOpMode() throws InterruptedException {
        chassis.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);

        // Camera things
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        ShippingElementRecognizer pipeline = new ShippingElementRecognizer();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        waitForStart();
        // Start button is pressed

        // Get the placement of the shipping element 100 times and pick the most frequent position
        int level;
        int[] counts = {0,0,0};
        for(int i=0;i<50;i++) {
            delay(10);
            if(pipeline.getShippingHubLevel() == 0) {
                i = 0;
                continue;
            }
            counts[pipeline.getShippingHubLevel() - 1]++;
        }
        if(counts[0] > counts[1] && counts[0] > counts[2]) {
            level = 1;
        } else if(counts[1] > counts[0] && counts[1] > counts[2]) {
            level = 2;
        } else {
            level = 3;
        }
        telemetry.addData("Shipping Hub Level", level);
        telemetry.update();

        // Drive to the the shipping hub
        chassis.moveBackwardWithEncoders(0.6,300);
        delay(250);
        chassis.strafeRightWithEncoders(0.6,100);
        chassis.rotate(30,0.5);
        chassis.moveBackwardWithEncoders(0.6,800);
        delay(250);

        // Deposit the box on the correct level
        if(level == 1) {
            lift.goTo(450,0.8);
            delay(300);
        } else if (level == 2) {
            lift.goTo(900,0.8);
            delay(400);
        } else {
            lift.goTo(1450, 0.8);
            delay(600);
        }
        hopper.hopper.setPosition(0.33);
        delay(1200);
        hopper.hopper.setPosition(0);
        delay(200);
        lift.goTo(0,0.8);

        // Drive to the warehouse
        chassis.moveForwardWithEncoders(0.6,800);
        chassis.rotate(50,0.5);
        chassis.strafeRightWithEncoders(0.6,800);

        // Get a piece of cargo
        intake.intakeMotor.setPower(0.8);
        chassis.moveForwardWithEncoders(0.6, 1900);
        delay(500);
        intake.intakeMotor.setPower(0);

        // Place the cargo
        chassis.moveBackwardWithEncoders(1,200);
        delay(200);
        chassis.strafeRightWithEncoders(0.3,200);
        chassis.strafeLeftWithEncoders(0.3,10);
        chassis.moveBackwardWithEncoders(0.6,1900);
        chassis.strafeLeftWithEncoders(0.6,350);
        chassis.rotate(-60,0.5);
        chassis.moveBackwardWithEncoders(0.6,800);
        lift.goTo(1450,0.8);
        delay(600);
        hopper.hopper.setPosition(0.33);
        delay(1200);
        hopper.hopper.setPosition(0);
        delay(200);
        lift.goTo(0,0.8);

        // Drive back into the warehouse
        chassis.moveForwardWithEncoders(0.6,800);
        chassis.rotate(60,0.5);
        chassis.strafeRightWithEncoders(0.6,800);
        chassis.moveForwardWithEncoders(0.6, 2100);
    }

    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time) {
        }
    }
}
