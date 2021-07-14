package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;


@Autonomous(name = "Auto", group = "Mechanum")
public class BasicAutonomous extends LinearOpMode {

    DcMotor frontLeft,backLeft,frontRight,backRight;

    ElapsedTime t;

    public static double TICKS_PER_CM = 17.83; // 17.83 tics/cm traveled(Strafer)
    public static double WHEEL_POWER = 0.6;
    public static double MOVE_CORRECTION = 1.0;
    public static double ROTATION_CORRECTION = 1.0; //(62/90);
    public static double STRAFE_CORRECTION = 1.0;


    //Ticks per revolution = 537.6(same for both)
    //wheel size is 100mm and circumference ~31.415 cm(regular)
    //wheel size is 96mm and circumference~30.15 cm(Strafer chassis)

    public void runOpMode() {

        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        t = new ElapsedTime();

        moveStraight(100);

        strafeRight(100);

        moveStraight(-100);

        strafeLeft(100);
        sleep(1000);
    }

    public void moveStraight(double distance) {
        backLeft.setTargetPosition((int) (distance * TICKS_PER_CM * MOVE_CORRECTION)); //ticks
        frontLeft.setTargetPosition((int) (distance * TICKS_PER_CM * MOVE_CORRECTION));
        frontRight.setTargetPosition((int) (distance * TICKS_PER_CM * MOVE_CORRECTION));
        backRight.setTargetPosition((int) (distance * TICKS_PER_CM * MOVE_CORRECTION));
        move();
    }

    public void strafeLeft(double distance) {
        backLeft.setTargetPosition((int) (distance * TICKS_PER_CM * STRAFE_CORRECTION)); //ticks
        frontLeft.setTargetPosition((int) (-distance * TICKS_PER_CM * STRAFE_CORRECTION));
        frontRight.setTargetPosition((int) (distance * TICKS_PER_CM * STRAFE_CORRECTION));
        backRight.setTargetPosition((int) (-distance * TICKS_PER_CM * STRAFE_CORRECTION));
        move();
    }
    public void strafeRight(double distance) {
        backLeft.setTargetPosition((int) (-distance * TICKS_PER_CM * STRAFE_CORRECTION)); //ticks
        frontLeft.setTargetPosition((int) (distance * TICKS_PER_CM * STRAFE_CORRECTION));
        frontRight.setTargetPosition((int) (-distance * TICKS_PER_CM * STRAFE_CORRECTION));
        backRight.setTargetPosition((int) (distance * TICKS_PER_CM * STRAFE_CORRECTION));
        move();
    }
    public void turnLeft(double distance) {
        backLeft.setTargetPosition((int) (-distance * TICKS_PER_CM * ROTATION_CORRECTION)); //ticks
        frontLeft.setTargetPosition((int) (-distance * TICKS_PER_CM * ROTATION_CORRECTION));
        frontRight.setTargetPosition((int) (distance * TICKS_PER_CM * ROTATION_CORRECTION));
        backRight.setTargetPosition((int) (distance * TICKS_PER_CM * ROTATION_CORRECTION));
        move();
    }
    public void turnRight(double distance) {
        backLeft.setTargetPosition((int) (distance * TICKS_PER_CM * ROTATION_CORRECTION)); //ticks
        frontLeft.setTargetPosition((int) (distance * TICKS_PER_CM * ROTATION_CORRECTION));
        frontRight.setTargetPosition((int) (-distance * TICKS_PER_CM * ROTATION_CORRECTION));
        backRight.setTargetPosition((int) (-distance * TICKS_PER_CM * ROTATION_CORRECTION));
        move();
    }
    public void move(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(WHEEL_POWER);
        backLeft.setPower(WHEEL_POWER);
        frontRight.setPower(WHEEL_POWER);
        backRight.setPower(WHEEL_POWER);

        while (opModeIsActive() && (backRight.isBusy() || backLeft.isBusy() || frontLeft.isBusy() || frontRight.isBusy()))
        {
            telemetry.addData("Time", t.seconds());
            telemetry.addData("encoder-bck-left", backLeft.getCurrentPosition() + " power= " + backLeft.getPower() +  "  busy=" + backLeft.isBusy());
            telemetry.addData("encoder-bck-right", backRight.getCurrentPosition() + " power= " + backRight.getPower() +  "  busy=" + backRight.isBusy());
            telemetry.addData("encoder-fwd-left", frontLeft.getCurrentPosition() + " power= " + frontLeft.getPower() +  "  busy=" +frontLeft.isBusy());
            telemetry.addData("encoder-fwd-right", frontRight.getCurrentPosition() + " power= " + frontRight.getPower() +  "  busy=" + frontRight.isBusy());

            telemetry.update();

        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        resetStartTime();


    }

    protected void log(String caption, Object value) {
        telemetry.addData(caption, value);
        RobotLog.a("[BBTC] " + caption + ": " + value);
    }
}