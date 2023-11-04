package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous
public class AprilTagTest extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
    //GyroSensor gyro;
    BNO055IMU imu;
    ColorSensor colorSensor;
    Servo backServo;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode(){
        /*
        boolean virtual = false;
        if(virtual) {
            m1 = hardwareMap.dcMotor.get("back_left_motor");
            m2 = hardwareMap.dcMotor.get("front_left_motor");
            m3 = hardwareMap.dcMotor.get("front_right_motor");
            m4 = hardwareMap.dcMotor.get("back_right_motor");
        } else {
            m1 = hardwareMap.dcMotor.get("leftBack");
            m2 = hardwareMap.dcMotor.get("leftFront");
            m3 = hardwareMap.dcMotor.get("rightFront");
            m4 = hardwareMap.dcMotor.get("rightBack");
        }
        m1.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //*/
        // Distance sensors are 4.9 cm from the robot's edge
        /*
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        backServo = hardwareMap.servo.get("back_servo");
        */

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal  = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                //.setCameraResolution(new Size(640, 480)) // not working for some reason
                .build();

        Orientation orientation;

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("April tags detected", "%d", tagProcessor.getDetections().size());
            for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                AprilTagDetection tag = tagProcessor.getDetections().get(i);
                telemetry.addData("April tag detection " + (i + 1), "X:" + (float) Math.round(tag.ftcPose.x * 100)/100 +", Y:" + (float) Math.round(tag.ftcPose.y * 100)/100 + ", Z:" + (float) Math.round(tag.ftcPose.z * 100)/100);
            }
            telemetry.update();
        }

        /*
        ElapsedTime waitTime = new ElapsedTime();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Seconds since init","%d. Press start when ready.", (int)waitTime.seconds());
            telemetry.update();
        }

        //Move forward for x seconds
        //setPower(0, 0.5f, 0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3.0) {
            //telemetry.addData("Front Distance: ", "%4.1f cm away", frontDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("Moving", " %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        setPower(0, 0, 0);
        //System.out.println(frontDistance.getDistance(DistanceUnit.CM));
        //*/


/*
        //Turn 45 degrees.
        setPower(0, 0, 0.5f);
        //while (opModeIsActive() && gyro.getHeading() < 45) continue;
        while (opModeIsActive()){
            orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (orientation.firstAngle >= 45) break;
        }
        setPower(0, 0, 0);

        float[] hsv = new float[3];
        //Drive forward until color sensor touches tape.
        setPower(0,0.5f,0);
        while (opModeIsActive()) {
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
            if (hsv[1] > 0.75) break;
        }

        //Keep driving forward until color sensor is off of tape.
        while (opModeIsActive()) {
            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
            if (hsv[1] < 0.25) break;
        }

        float currentHeading = 45;
        ElapsedTime et = new ElapsedTime();

        //Main loop; once per trip around the lander
        while (opModeIsActive()){

            //Turn 90 degrees.
            setPower(0,0,0.5f);
            backServo.setPosition(0.333 * (currentHeading - 45.0f)/90.0f);
            telemetry.addData("Turning 90 degrees","");
            telemetry.update();

            //while (opModeIsActive() && AngleUtils.normalizeDegrees(gyro.getHeading() - currentHeading) < 90) continue;

            while (opModeIsActive()){
                orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (AngleUtils.normalizeDegrees(orientation.firstAngle - currentHeading) >= 90) break;
            }

            currentHeading = (float)AngleUtils.normalizeDegrees360(currentHeading + 90);

            //Follow outside edge of tape until color sensor has not seen tape for 0.5 seconds.
            telemetry.addData("Line Following","");
            telemetry.update();
            et.reset();
            while (opModeIsActive() && et.seconds() < 0.5) {
                Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
                if (hsv[1] > 0.25) et.reset();
                setPower( 2.0f * (hsv[1] - 0.5f), 0.5f, 0);
            }

        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        */
    }
/*
    void setPower(float px, float py, float pa){
        double p1 = -px + py - pa;
        double p2 = px + py + -pa;
        double p3 = -px + py + pa;
        double p4 = px + py + pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        m1.setPower(p1);
        m2.setPower(p2);
        m3.setPower(p3);
        m4.setPower(p4);
    }
    //*/
}
