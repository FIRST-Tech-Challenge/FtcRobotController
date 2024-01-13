package org.firstinspires.ftc.teamcode.alex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "Auto (Scorpion)", group = "Autonomous")
@Disabled
public class autoRCD extends LinearOpMode {

    private Servo leftGrip;
    private Servo rightGrip;
    private DcMotor armRotate;
    private DcMotor armExt;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {

        //Initialize Variables
        //Tracks rotation of the arm's motor
        int rotation;
        //Tracks the extension of the arm
        int ext;
        //X and Y values of stick inputs to compile drive outputs
        float y;
        double x;

        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //Sets initial positions and directions for grip servos
        leftGrip.setDirection(Servo.Direction.REVERSE);
        rightGrip.setDirection(Servo.Direction.FORWARD);
        leftGrip.setPosition(0.75);
        rightGrip.setPosition(0.75);

        //Sets variables to 0 on initialization
        rotation = 0;
        ext = 0;
        waitForStart();

        if (opModeIsActive()) {

            //Sets behaviors and modes for motors
            //ArmExtension and ArmRotate are set to brake when receiving zero power
            //Arm Extension is set to run using encoder outputs and inputs
            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive()) {

                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);
                sleep(500);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);

                //Telemetry for debugging
                telemetry.addData("Current Arm Extension", ext);
                telemetry.addData("Current Arm Rotation", rotation);
                telemetry.update();
            }
        }
    }
}
