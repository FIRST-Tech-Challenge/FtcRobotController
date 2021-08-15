package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417_2020.Resources.Toggler;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="2 Goal Auto")
public class TwoWobbleGoalAutoComp extends MasterAutonomous {

    int allianceSide = 1;
    Toggler rightBumper = new Toggler();

    // Constants
    static final int MINIMUM_RING_AREA = 1800;
    static final int PARKING_Y_POSITION = -72;
    static final int A_C_TARGET_ZONE_X_POSITION = 11;
    static final int A_TARGET_ZONE_Y_POSITION = -68;
    static final int B_TARGET_ZONE_Y_POSITION = -93;
    static final int C_TARGET_ZONE_Y_POSITION = -113;

    RingDetectionOpenCV ringDetector = new RingDetectionOpenCV();
    OpenCvCamera webcam;
    int numRings;

    @Override
    public void runOpMode() throws InterruptedException {
        autoInitializeRobot();
        initializeRingCounter();

        telemetry.addLine("waiting for start");

        while (!opModeIsActive()) {
            countRings();
            pickAllianceSide();
            telemetry.update();
        }


        waitForStart();

        //strafe away from wall and pivot
        move(-6, 0, 0.9);
        pivot(-90, 0.6);
        // 0 rings -> Zone A
        // 1 rings -> Zone B
        // 4 rings -> Zone C
        deliverWobbleGoal();


        //move(11, 2, 0.9);
        // navigate to other wobble goal by going back towards wall and strafing
        switch (numRings) {
            case 0:
                move(11, 2, 0.9);
                move(-21, 2, 0.9);
                break;
            case 1:
                move(11, 1, 0.9);
                move(-22, 1, 0.9);
                break;
            case 4:
                move(11, 2, 0.9);
                move(-22, 4, 0.7);
                break;
        }


        // going forward to catch wobble goal
        move(-22, -8, 0.3);


        // deliver second wobble goal
        switch (numRings) {
            case 0:
                move(13, -64, 0.9);
                move(13,-47, 0.9);
                // navigate to shoot
                move(-18, -60, 0.4);
                pivot(92,0.4);

                // shoot left power shot
                shootLauncher(-0.8);
                pivot(84, 0.4);
                shootLauncher(-0.85);
                pivot(78, 0.4);
                shootLauncher(-0.85);
                motorLauncher.setPower(0.0);
                break;

            case 1:
                // Navigate to Zone B
                move(-35, -41, 0.9);
                move(-12, -90, 0.9);
                //OpenWobbleGoalGrabber();
                // navigate to shoot
                move(-22, -58, 0.4);
                pivot(92,0.4);

                // shoot left power shot
                shootLauncher(-0.85);
                pivot(84, 0.4);
                shootLauncher(-0.80);
                pivot(78, 0.4);
                shootLauncher(-0.85);
                motorLauncher.setPower(0.0);

                break;

            case 4:
                // Navigate to Zone C
                move(-33, -41, 0.9);
                move(A_C_TARGET_ZONE_X_POSITION, -111, 0.9);
                //OpenWobbleGoalGrabber();
                // navigate to shoot
                move(-21, -60, 0.9); //-19
                pivot(92,0.6);

                // shoot left power shot
                shootLauncher(-0.85);
                pivot(83, 0.4);
                shootLauncher(-0.85);
                pivot(78, 0.4);
                shootLauncher(-0.85);
                motorLauncher.setPower(0.0);
                break;

        }




        // parking
        move(-18, -48, 0.9);



        /*
        wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_OUT);
        runMotorToPosition(motorWobbleGoalArm, -250, 0.5);

        sleep(300);
        // close the grabber
        wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_IN);
        */

    }

    // todo program Wobble Goal arm
    public void OpenWobbleGoalGrabber() {
        //wobbleGoalGrabber.setPosition(WOBBLE_GOAL_GRABBER_OUT);

    }

    // Uses gamepad to pick alliance side
    public int pickAllianceSide() {
        boolean isRedSide = rightBumper.toggle(gamepad1.right_bumper);
        if (isRedSide) {
            allianceSide = 1;
            telemetry.addLine("--------RED ALLIANCE--------");
        } else {
            allianceSide = -1;
            telemetry.addLine("--------BLUE ALLIANCE--------");
        }
        return allianceSide;
    }

    // Determines number of rings based on area and aspect ratio of orange rectangle
    public int countRings() {
        telemetry.addData("max rectangle", ringDetector.maxRect.toString());

        // area has to be greater than 1800, otherwise there are 0 rings
        if (ringDetector.maxRect.area() < MINIMUM_RING_AREA) {
            numRings = 0;
        }
        else if (ringDetector.maxRect.width / ringDetector.maxRect.height > 2.5) {
            numRings = 1;
        } else {
            numRings = 4;
        }
        telemetry.addData("Number of rings", numRings);

        // Use gamepad to adjust U threshold for filtering orange in different lighting conditions
        if (gamepad1.a) {
            ringDetector.uThreshold ++;
            sleep(200);
        } else if (gamepad1.b) {
            ringDetector.uThreshold --;
            sleep(200);
        }
        telemetry.addData("U Threshold", ringDetector.uThreshold);

        return numRings;
    }

    // Set up webcam streaming for counting Rings
    public void initializeRingCounter() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Side Webcam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        webcam.setPipeline(ringDetector);
    }

    public void deliverWobbleGoal() throws InterruptedException {
        switch (numRings) {
            case 0:
                // Nagivate to Zone A
                move(A_C_TARGET_ZONE_X_POSITION, -24, 0.9);
                move(A_C_TARGET_ZONE_X_POSITION, A_TARGET_ZONE_Y_POSITION, 0.9);
                OpenWobbleGoalGrabber();
                move(A_C_TARGET_ZONE_X_POSITION, (A_TARGET_ZONE_Y_POSITION + 15), 0.9);
                break;

            case 1:
                // Navigate to Zone B
                move(A_C_TARGET_ZONE_X_POSITION, -24, 0.9);
                move(-12, B_TARGET_ZONE_Y_POSITION, 0.9);
                OpenWobbleGoalGrabber();
                move(-12, -85, 0.9);
                move(11, -85, 0.9);
                break;

            case 4:
                // Navigate to Zone C
                move(A_C_TARGET_ZONE_X_POSITION, -24, 0.9);
                move(A_C_TARGET_ZONE_X_POSITION, C_TARGET_ZONE_Y_POSITION, 0.9);
                OpenWobbleGoalGrabber();
                break;

        }


    }

    public void runMotorForSeconds(int milliseconds, double power) {
        motorWobbleGoalArm.setPower(power);
        sleep(milliseconds);
        motorWobbleGoalArm.setPower(0.0);

    }


    public void shootLauncher(double speed) {
        servoRamp.setPosition(RAMP_UP);
        motorLauncher.setPower(speed);
        sleep(300);
        sleep(1100);
        servoRingPusher.setPosition(RING_PUSHER_IN);
        sleep(200);
        servoRingPusher.setPosition(RING_PUSHER_OUT);
    }



}
