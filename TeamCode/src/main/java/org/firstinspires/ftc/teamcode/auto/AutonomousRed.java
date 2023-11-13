package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "red auto")
public class AutonomousRed extends LinearOpMode {

    protected DcMotor left_front;
    protected DcMotor right_front;
    protected DcMotor left_back;
    protected DcMotor right_back;
    protected DcMotor arm;
    protected Servo servo;
    private DcMotor intake = null;
    OpenCvCamera camera;
    PipelineRed pipeline = new PipelineRed();

    enum propPosition {
            LEFT,
            MIDDLE,
            RIGHT
    }


    @Override
    public void runOpMode() throws InterruptedException {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");

        arm = hardwareMap.get(DcMotor.class, "arm");

        intake = hardwareMap.get(DcMotor.class, "intake");

        servo = hardwareMap.get(Servo.class, "servo");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //right_front.setPower(1);
        //left_front.setPower(1);
        //right_back.setPower(1);
        //left_back.setPower(1);
        //sleep(1000);
        //right_front.setPower(0);
        //left_front.setPower(0);
        //right_back.setPower(0);
        // left_back.setPower(0);

        arm.setPower(0.075);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ensure pixel is in right side of box!!");
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());
            telemetry.update();
        }
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0.075);
        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();

        moveForward(0.25, 2200);

        if (propArea < 10000) { // none detected we assume left spike mark
            telemetry.addLine("Left spike mark");
            telemetry.update();
            moveTurning(-0.25, 2200);
            moveForward(0.5, 100);
            // eject pixel
            intake.setPower(-1);
            sleep(1000);
            intake.setPower(0);
            moveForward(0.25, 100);
            moveForward(-0.25, 100);
            // go to backboard and score, evading pixel


        } else if (propX > 600) { // right spike mark
            telemetry.addLine("Right spike mark");
            telemetry.update();
            // line up with mark
            moveTurning(0.25, 2200);
            moveForward(0.25, 300);
            // eject pixel
            intake.setPower(-0.25);
            sleep(1000);
            intake.setPower(0);
            //in case it falls vertically
            moveForward(0.25, 100);
            moveForward(-0.25, 100);
            // go to backboard and score
        } else { // middle spike mark
            telemetry.addLine("Middle spike mark");
            telemetry.update();
            moveForward(0.25, 400);
            moveForward(-0.25, 400);
            // eject pixel- we're already there
            intake.setPower(-0.25);
            sleep(1000);
            intake.setPower(0);
            moveForward(0.25, 100);
            moveForward(-0.25, 100);
            // go to backboard
        }

        servo.setPosition(0);
        sleep(400);
        arm.setTargetPosition(700);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0/8);
        sleep(1000);
        servo.setPosition(0.7);

        sleep(100000);
    }

    public void moveForward(double power, int time) {
        right_front.setPower(power);
        left_front.setPower(power);
        right_back.setPower(power);
        left_back.setPower(power);
        sleep(time);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);
        left_back.setPower(0);

    }

    public void moveStrafing(double power, int time){
        right_front.setPower(power);
        left_front.setPower(power);
        right_back.setPower(-power);
        left_back.setPower(-power);
        sleep(time);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);
        left_back.setPower(0);

    }

    public void moveTurning(double power, int time){
        right_front.setPower(-power);
        left_front.setPower(power);
        right_back.setPower(-power);
        left_back.setPower(power);
        sleep( time);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);
        left_back.setPower(0);
    }



   /*
    Strafing:
    lf +
    rf +
    lb -
    rb -
    Turning
    lf +
    rf -
    lb +
    rb -
     */

    //Drive the robot forward
}
