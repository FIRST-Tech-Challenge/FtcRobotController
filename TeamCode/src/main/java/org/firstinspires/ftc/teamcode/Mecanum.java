package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Mecanum extends LinearOpMode {
    private DcMotor frontRight= null; //java sucks
    private DcMotor frontLeft= null; //python lowk better
    private DcMotor backRight= null; //we should use py
    private DcMotor backLeft= null; //83


    private String frontRightconfig=""; //add configs when phones exist
    private String frontLeftconfig="";
    private String backRightconfig="";
    private String backLeftconfig="";
    private Gamepad gamepad1 = new Gamepad(); //constructor
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotor .class, frontRightconfig);
        frontLeft = hardwareMap.get(DcMotor.class, frontLeftconfig);
        backRight = hardwareMap.get(DcMotor.class, backRightconfig);
        backLeft = hardwareMap.get(DcMotor.class, backLeftconfig);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait to press START on controller
        waitForStart();

        while (opModeIsActive()) {
            double JOYSTICK_MIN;
            double JOYSTICK_MAX;

            double y  = -gamepad1.left_stick_y;
            double rx = gamepad1.left_stick_x;
            double power = Math.sqrt(Math.pow(y,2)+Math.pow(rx,2)); //java sucks
            double x = Math.atan2(y, rx);

            if (gamepad1.right_stick_x != 0 ){
                frontRight.setPower(x - (1/4) * Math.PI);
                backLeft.setPower(x - (1/4) * Math.PI);

                frontLeft.setPower(x + (1/4) * Math.PI);
                backRight.setPower(x + (1/4) * Math.PI);
            } else {
                frontLeft.setPower(x);
                backLeft.setPower(x);

                frontRight.setPower(-x);
                backRight.setPower(-x);
            }
        }
    }


}