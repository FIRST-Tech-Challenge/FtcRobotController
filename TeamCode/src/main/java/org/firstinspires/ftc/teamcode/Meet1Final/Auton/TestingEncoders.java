package org.firstinspires.ftc.teamcode.Meet1Final.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Testing")
public class TestingEncoders extends LinearOpMode {

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private double Ticks_Per_Inch = 45.2763982107824;

    private int leftFrontDriveTickTracker = 0;
    private int rightFrontDriveTickTracker = 0;
    private int leftBackDriveTickTracker = 0;
    private int rightBackDriveTickTracker = 0;

    @Override
    public void runOpMode() {

        DcMotor rightBackDrive = hardwareMap.dcMotor.get("motorBR");

        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int liftTargetPosition = 0;
        double openClaw = 0.4; //increase to open more
        double closeClaw = 0.2;


        waitForStart();

        if (opModeIsActive()) {
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            rightBackDrive.setTargetPosition(1450);
            rightBackDrive.setPower(0.5);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
            telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

            telemetry.update();

            while (rightBackDrive.isBusy()) {}

            rightBackDrive.setPower(0);

            }

            int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(31);

            int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;

            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() - rightBackDriveTargetTicks);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setPower(0.5);

            telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
            telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

            telemetry.update();

            while (rightBackDrive.isBusy()) {}

            rightBackDrive.setPower(0);



    }

    public int calculateTicksForLateralMovement(double inches) {
        double Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Rounded_Encoder_Ticks;
    }
}
