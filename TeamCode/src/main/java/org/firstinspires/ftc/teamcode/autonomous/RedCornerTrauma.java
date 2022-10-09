package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="RedCorner", group="Trauma")
public class RedCornerTrauma extends DriveDirections {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private double moveSpeed = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Pushes cone onto junction
        DriveForDistance(moveSpeed, .52, "LEFT");
        DriveForDistance(moveSpeed, .08, "BACKWARD");
        DriveForDistance(moveSpeed, .29, "FORWARD");
        DriveForDistance(moveSpeed, .3, "BACKWARD");
        DriveForDistance(moveSpeed, .36, "RIGHT");
        DriveForDistance(moveSpeed, .75, "FORWARD");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

