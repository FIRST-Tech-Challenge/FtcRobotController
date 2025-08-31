package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Robot;

// If we're using diffy then this would have to be rewritten a bit but otherwise usefull
// ----- READY TO TRANSFER ----- //
@Config
@TeleOp(name = "Angle Tuner")
public class AngleTuner extends LinearOpMode {

    Robot robot;
    public static double currentHeading;
    public static double desiredHeading;
    public static double velocity = 1;

    DcMotor frWheel, flWheel, brWheel, blWheel;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap);

        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        brWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive()){

            if (currentHeading == desiredHeading)
            {
                flWheel.setPower(0);
                frWheel.setPower(0);
                blWheel.setPower(0);
                brWheel.setPower(0);
            } else{
                if (desiredHeading <= 180 && desiredHeading >= -180) {
                    moveToDesiredAngle(desiredHeading, currentHeading);
                }
            }
            currentHeading = robot.getDrivetrain().getRobotHeading(AngleUnit.RADIANS);
            telemetry.addData("Current heading: ", robot.getDrivetrain().getRobotHeading(AngleUnit.RADIANS));
        }
    }


    public void moveToDesiredAngle(double desiredHeading, double currentHeading)
    {
        if (desiredHeading > currentHeading)
        {
            clockWise(velocity);
        }

        if (desiredHeading < currentHeading)
        {
            counterClockWise(velocity);
        }
    }

    public void clockWise(double velocity)
    {
        flWheel.setPower(velocity);
        frWheel.setPower(-velocity);
        blWheel.setPower(velocity);
        brWheel.setPower(-velocity);
    }

    public void counterClockWise(double velocity)
    {
        flWheel.setPower(-velocity);
        frWheel.setPower(velocity);
        blWheel.setPower(-velocity);
        brWheel.setPower(velocity);
    }
}
