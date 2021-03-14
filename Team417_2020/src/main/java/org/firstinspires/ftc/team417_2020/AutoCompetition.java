package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417_2020.Resources.Toggler;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Competition Auto")
public class AutoCompetition extends MasterAutonomous{

    int allianceSide = 1;
    Toggler rightBumper = new Toggler();

    RingDetectionOpenCV ringDetector = new RingDetectionOpenCV();
    OpenCvCamera webcam;
    int numRings;



    @Override
    public void runOpMode() throws InterruptedException {
        autoInitializeRobot();
        initializeRingCounter();


        telemetry.addLine("waiting for start");
        telemetry.update();

        while (!opModeIsActive()) {
            countRings();
            pickAllianceSide();
            telemetry.update();
        }


        waitForStart();

        // 0 rings -> Zone A
        // 1 rings -> Zone B
        // 4 rings -> Zone C
        if (numRings == 0) {
            // Nagivate to Zone A
            move(0, 72, 0.7);
            move(-24, 72, 0.7);
            openWobbleGoalArm();
            move(-24, 48, 0.7);
            move(0, 84, 0.7);

        } else if (numRings == 1) {
            // Navigate to Zone B
            move(0, 96, 0.7);
            openWobbleGoalArm();
            move(0, 84, 0.7);

        } else if (numRings == 4) {
            // Navigate to Zone C
            move(0, 120, 0.7);
            move( -24, 120, 0.7);
            openWobbleGoalArm();
            move(0, 84, 0.7);

        }

    }

    public void openWobbleGoalArm() {

    }

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

    public int countRings() {
        telemetry.addData("max rectangle", ringDetector.maxRect.toString());

        // area at 30 inches away (for threshold of 0 rings):

        // 4 rings area: 105x75
        // 1 ring area: 105x23

        // area has to be greater than 2000, otherwise there are 0 rings

        if (ringDetector.maxRect.area() < 1800) {
            numRings = 0;
        }
        else if (ringDetector.maxRect.width / ringDetector.maxRect.height > 2.5) {
            numRings = 1;
        } else {
            numRings = 4;
        }
        telemetry.addData("Number of rings", numRings);

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

    public void initializeRingCounter() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
}
