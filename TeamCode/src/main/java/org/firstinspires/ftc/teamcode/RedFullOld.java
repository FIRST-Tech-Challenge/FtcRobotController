
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Robot.*;


@Autonomous(name = "Old Autonomous")
public class RedFullOld extends LinearOpMode {

    OpenCvCamera webcam;
    final int ARM_DOWN = 80;
    final int ARM_UP = 0;
    final double ARM_POWER = 1;

    @Override
    public void runOpMode() {

////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        Robot.initMotors(this);
        Robot.initIMU(this);

        //////Start Camera Streaming//////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);


        RingVisionPipeline pipeline = new RingVisionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });


////////Program start////////////////////////////////////////////////////////////////////////
        //init wobble servos
        wobbleArm.setPosition(1);
        wobbleClaw.setPosition(0);

        waitForStart();
        telemetry.addData("location: ", pipeline.getLocation());
        telemetry.update();

        //store the value of the ring stack
        int stackPos;
        switch (pipeline.getLocation()) {
            case C_FULL_STACK:
                stackPos = 3;
                break;
            case B_HALF_STACK:
                stackPos = 2;
                break;
            case A_NO_STACK:
                stackPos = 1;
                break;
            default: stackPos = 1;
        }

        //make motors float rather than full stop to ensure robot stays straight for power shots
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //move to launch location
        forward(59,0.4);
        sleep(400);

        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //launch ring1
        launcher1.setPower(0.56);
        launcher2.setPower(-0.56);
        sleep(1000);
        launcherbelt.setTargetPosition(900);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.6);
        while(launcherbelt.isBusy()) {}

        //launch ring2
        counter(1.67,0.5);
        launcher1.setPower(0.61);
        launcher2.setPower(-0.61);
        sleep(600);
        launcherbelt.setTargetPosition(1800);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.6);
        while(launcherbelt.isBusy()) {}

        //launch ring3
        counter(1.9,0.5);
        launcher1.setPower(0.59);
        launcher2.setPower(-0.59);
        sleep(600);
        launcherbelt.setTargetPosition(3100);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(0.6);
        while(launcherbelt.isBusy()) {}
        launcher1.setPower(0);
        launcher2.setPower(0);



        switch (stackPos) {
            case 3:
                //FULL Stack rings
                moveToAngle(137,0.3);
                back(56,0.7);
                break;
            case 2:
                //HALF stack rings
                moveToAngle(148,0.3);
                back(23,0.7);
                break;
            case 1:
                //NO rings
                moveToAngle(105,0.3);
                back(22,0.7);
        }

        ejectWobbleGoal();

        //go get second wobble goal
        switch (stackPos) {
            case 3:
                //FUll stack rings
                moveToAngle(0,.35);
                right(15,.5); //align robot against wall
                sleep(100);
                back(65,1);
                pickUpWobble();
                moveToAngle(-179,.4); //face drop zone C
                back(62,1);
                ejectWobbleGoal();
                forward(22,1);
                break;
            case 2:
                //Half stack rings
                moveToAngle(0,.3);
                right(43,.5); //align robot against wall
                sleep(100);
                back(38,1);
                pickUpWobble();
                moveToAngle(-160,.4); //face drop zone B
                back(39,1);
                ejectWobbleGoal();
                forward(5,0.8);
                break;
            case 1:
                //No rings
                moveToAngle(0,0.3);
                right(29,.4);
                back(28,1);
                pickUpWobble();
                moveToAngle(-185,0.40); //face drop zone A
                back(27,0.8);

                ejectWobbleGoal();
                /*
                //eject wobble but leave arm down
                wobbleArm.setPosition(.75);
                wobbleArmMotor.setTargetPosition(ARM_DOWN - (ARM_DOWN/2) );
                wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArmMotor.setPower(ARM_POWER);
                sleep(900);
                wobbleClaw.setPosition(1);
                */


        }

        //set launcher belt to starting position (not necessary but helps when running auto repetitively for testing)
        launcherbelt.setTargetPosition(0);
        launcherbelt.setPower(0.6);
        while(launcherbelt.isBusy()) {}
    }


    private void ejectWobbleGoal() {
        wobbleArm.setPosition(.5); //down
        wobbleArmMotor.setTargetPosition(ARM_DOWN);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        sleep(900);
        wobbleClaw.setPosition(1);
        sleep(300);
        wobbleArm.setPosition(1); //up
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        sleep(500);
        wobbleClaw.setPosition(0);
    }

    private void pickUpWobble(){
        wobbleArm.setPosition(.5); //down
        wobbleArmMotor.setTargetPosition(ARM_DOWN);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        sleep(700);
        wobbleClaw.setPosition(1);
        sleep(700);
        moveToAngle(-45,.3);
        wobbleClaw.setPosition(0);
        sleep(1000);
        wobbleArm.setPosition(1); //up
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        sleep(700);
    }

    private void launchRingPri(double belt, double speed) {
        launcher1.setPower(speed);
        launcher2.setPower(-speed);
        sleep(1750);
        launcherbelt.setPower(0.5);
        sleep((long) (belt*1000));
        launcher1.setPower(0);
        launcher2.setPower(0);
        launcherbelt.setPower(0);

    }
}