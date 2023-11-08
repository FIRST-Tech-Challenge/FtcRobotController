package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.auto.BasicPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "da not a moose")
public class Autonomous extends LinearOpMode {

    protected DcMotor left_front;
    protected DcMotor right_front;
    protected DcMotor left_back;
    protected DcMotor right_back;
    OpenCvCamera camera;
    BasicPipeline pipeline = new BasicPipeline();


    @Override
    public void runOpMode() throws InterruptedException {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right_back");
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

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        CoolestDrive(1000, 1000, 0, 1, true);
    }

    public void CoolestDrive(double forward, double strafe, double yaw, double speed, boolean waitToFinish){
        left_front.setTargetPosition((int) (forward + strafe + yaw));
        left_back.setTargetPosition((int) (forward - strafe - yaw));
        right_front.setTargetPosition((int) (forward - strafe + yaw));
        right_back.setTargetPosition((int) (forward + strafe - yaw));

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double leftFrontPower = speed * (forward + strafe + yaw);
        double rightFrontPower = speed * (forward - strafe - yaw);
        double leftBackPower = speed * (forward - strafe + yaw);
        double rightBackPower = speed * (forward + strafe - yaw);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        left_front.setPower(leftFrontPower);
        left_back.setPower(leftBackPower);
        right_front.setPower(rightFrontPower);
        right_back.setPower(rightBackPower);

        if (waitToFinish) while (left_front.isBusy()) telemetry.addLine("Running coolest drive"); telemetry.update();
    }

    public void moveForward(int power, int time) {
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

    public void moveStrafing(int power, int time){
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

    public void moveTurning(int power, int time){
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
