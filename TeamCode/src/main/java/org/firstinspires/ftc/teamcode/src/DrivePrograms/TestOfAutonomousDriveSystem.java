package org.firstinspires.ftc.teamcode.src.DrivePrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.AutonomousDriveSystem;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.OdometryPodServos;
import org.firstinspires.ftc.teamcode.Executable;
import org.firstinspires.ftc.teamcode.src.robotAttachments.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.OdometryDrivetrain;


@Autonomous(name = "TestOfAutonomousDriveSystem")
public class TestOfAutonomousDriveSystem extends LinearOpMode {
    AutonomousDriveSystem driveSystem;

    public void runOpMode() throws InterruptedException {
        //HardwareMap hardwareMap, String front_right, String front_left, String back_right, String back_left, String horizontalEncoderName, String leftEncoderName,
        // String rightEncoderName,String horizontalServo, String leftVerticalServo,String rightVerticalServo, Executable<Boolean> isStopRequested,
        // Executable<Boolean> isOpModeActive
        /*
        driveSystem = new AutonomousDriveSystem(hardwareMap, "front_right", "front_left", "back_right", "back_left", "back_right", "front_left", "front_right", "horizontal_odometry_servo", "left_odometry_servo", "right_odometry_servo",
                new Executable<Boolean>() {
                    @Override
                    public Boolean call() {
                        return isStopRequested();
                    }
                }, new Executable<Boolean>() {
            @Override
            public Boolean call() {
                return opModeIsActive();
            }
        });
        driveSystem.start();
        driveSystem.lowerOdometryPods();
        driveSystem.setPosition(0,0,0);

        waitForStart();

        driveSystem.strafeAtAngle(45,45);
        ElapsedTime t = new ElapsedTime();
        while (t.seconds() < 1){}

        try{

            //driveSystem.moveToPosition(50,50,.5);
        }catch (NullPointerException error){
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            error.printStackTrace(pw);
            String exceptionString = sw.toString();
            while (opModeIsActive()){
                telemetry.addData("Error: ",exceptionString);
                telemetry.update();
            }
        }

        return;*

         */
        OdometryPodServos servos = new OdometryPodServos(hardwareMap, "right_odometry_servo", "left_odometry_servo", "horizontal_odometry_servo");
        servos.lower();

        DcMotor front_right = hardwareMap.dcMotor.get("front_right");
        DcMotor front_left = hardwareMap.dcMotor.get("front_left");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right");

        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);


//front_left, front_right, back_left
        OdometryGlobalCoordinatePosition odometry = new OdometryGlobalCoordinatePosition(front_left, front_right, back_left, 1892.3724283364, 25);
        Thread positionThread = new Thread(odometry);
        positionThread.start();

        OdometryDrivetrain driveSystem = new OdometryDrivetrain(front_right, front_left, back_right, back_left, telemetry, odometry, new Executable<Boolean>() {
            @Override
            public Boolean call() {
                return isStopRequested();
            }
        }, new Executable<Boolean>() {
            @Override
            public Boolean call() {
                return opModeIsActive();
            }
        });

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("x: ", odometry.returnXCoordinate());
            telemetry.addData("y: ", odometry.returnYCoordinate());
            telemetry.addData("rot", odometry.returnOrientation());
            telemetry.update();
        }
    }
}
