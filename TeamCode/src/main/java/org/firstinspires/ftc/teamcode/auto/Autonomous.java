package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.auto.BasicPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "da not a moose")
public class Autonomous extends LinearOpMode {

    protected DcMotorEx left_front;
    protected DcMotorEx right_front;
    protected DcMotorEx left_back;
    protected DcMotorEx right_back;
    ArrayList<DcMotorEx> driveMotors = new ArrayList<>();
    protected DcMotor arm;
    protected Servo servo;
    private DcMotor intake = null;
    OpenCvCamera camera;
    BasicPipeline pipeline = new BasicPipeline();

    enum propPosition {
            LEFT,
            MIDDLE,
            RIGHT
    }


    @Override
    public void runOpMode() throws InterruptedException {
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        driveMotors.add(left_front);
        driveMotors.add(left_back);
        driveMotors.add(right_front);
        driveMotors.add(right_back);

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

        for(DcMotorEx driveMotor: driveMotors) {
            driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        moveForward(0.25, 2400);

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
            // go to backboard and score
            moveForward(-0.25, 4000);
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
            // go to backboard
            moveForward(-0.25, 400);
            moveTurning(0.25, 2500);
            moveForward(-0.25, 3200);
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

    public void moveForward(double power, int setpoint) {
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor.setTargetPosition(setpoint);
            driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveMotor.setPower(power);
        }
        while (left_front.isBusy()) telemetry.addLine("going forward");
    }

    public void moveStrafing(double power, int setpoint){
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        left_front.setTargetPosition(setpoint);
        left_back.setTargetPosition(-setpoint);
        right_front.setTargetPosition(-setpoint);
        right_back.setTargetPosition(setpoint);
        for (DcMotorEx driveMotor : driveMotors) driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setPower(power);
        left_front.setPower(-power);
        right_back.setPower(-power);
        left_back.setPower(power);

    }

    public void moveTurning(double power, int setpoint){
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        left_front.setTargetPosition(setpoint);
        left_back.setTargetPosition(setpoint);
        right_front.setTargetPosition(-setpoint);
        right_back.setTargetPosition(-setpoint);
        for (DcMotorEx driveMotor : driveMotors) driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setPower(-power);
        left_front.setPower(power);
        right_back.setPower(-power);
        left_back.setPower(power);

        while (left_front.isBusy()) telemetry.addLine("going forward");
    }

}
