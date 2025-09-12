package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Robot;

// this will probably be usefull

@TeleOp(name = "Odometry Test")
public class Odemtry extends LinearOpMode {

    Robot robot;
    Drivetrain drivetrain;
    DcMotor frWheel, flWheel, brWheel, blWheel;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */

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

        frWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        /*
         * * * * * * * * * * * * * * *
         * Start button clicked
         * * * * * * * * * * * * * * *
         */
        telemetry.clear();
        robot.getDrivetrain().startOdometry();

        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */

        while(opModeIsActive()) {

            fieldCentricDrive();
            EditablePose2D currPos = robot.getDrivetrain().getCurrPos();

            telemetry.addData("X Position", currPos.getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", currPos.getY(DistanceUnit.INCH));
            telemetry.addData("Orientation (Degrees)", Math.toDegrees(currPos.getH()));

            telemetry.addLine("\n Raw Values \n")
                    .addData("leftEncoder", robot.getDrivetrain().getRobotPos().getLeftEncoder())
                    .addData("rightEncoder", robot.getDrivetrain().getRobotPos().getRightEncoder())
                    .addData("perpendicularEncoder", robot.getDrivetrain().getRobotPos().getPerpendicularEncoder());

            telemetry.addLine("\n IMU measured heading \n")
                    .addData("Orientation (Degrees)", robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES));

            telemetry.update();
        }
    }

    public void fieldCentricDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; // 1.1 is to account for hardware inconsistencies.
        double rx = gamepad1.right_stick_x;

        double heading = robot.getDrivetrain().getRobotHeading(AngleUnit.RADIANS); // heading of bot in radians

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading); // Linear transformations yay
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // we like our drivers to have more control
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        robot.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);
    }
}
