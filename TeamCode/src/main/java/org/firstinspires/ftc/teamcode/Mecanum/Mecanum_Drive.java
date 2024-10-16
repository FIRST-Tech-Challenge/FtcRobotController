package org.firstinspires.ftc.teamcode.Mecanum;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;


@TeleOp
public class Mecanum_Drive extends LinearOpMode {
    DcMotor FLMotor, FRMotor, BLMotor, BRMotor;

    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {


        initRobot();


        waitForStart();
        while (opModeIsActive()) {
            moveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        }

    }

    public void initRobot() {
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

        FLMotor.setMode(RUN_WITHOUT_ENCODER);
        FRMotor.setMode(RUN_WITHOUT_ENCODER);
        BLMotor.setMode(RUN_WITHOUT_ENCODER);
        BRMotor.setMode(RUN_WITHOUT_ENCODER);

        FLMotor.setDirection(REVERSE);
        FRMotor.setDirection(FORWARD);
        BLMotor.setDirection(REVERSE);
        BRMotor.setDirection(FORWARD);

        FLMotor.setZeroPowerBehavior(BRAKE);
        FRMotor.setZeroPowerBehavior(BRAKE);
        BLMotor.setZeroPowerBehavior(BRAKE);
        BRMotor.setZeroPowerBehavior(BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        odo.setOffsets(100,100);
    }


    public void moveRobot(double FBPower, double LRPower, double turnPower) {
        double FLPower = FBPower + LRPower + turnPower;
        double FRPower = FBPower - LRPower - turnPower;
        double BLPower = FBPower - LRPower + turnPower;
        double BRPower = FBPower + LRPower - turnPower;

        double maxPower = Math.max(Math.max(FLPower, FRPower), Math.max(BLPower, BRPower));
        if (maxPower > 1) {
            FLPower = FLPower / maxPower;
            FRPower = FRPower / maxPower;
            BLPower = BLPower / maxPower;
            BRPower = BRPower / maxPower;
        }

        FLMotor.setPower(FLPower);
        FRMotor.setPower(FRPower);
        BLMotor.setPower(BLPower);
        BRMotor.setPower(BRPower);

    }


    public void moveFieldCentric(double FBPower, double LRPower, double turnPower) {
        double[] desireMove = cartesianToPolar(LRPower,FBPower);
        double heading = odo.getHeading() - desireMove[1];
        desireMove = cartesianToPolar(desireMove[0],heading);

        moveRobot(desireMove[1],desireMove[0],turnPower);
    }


    // Converts cartesian coordinates to polar
    // 0 = r
    // 1 = theta
    public double[] cartesianToPolar(double x, double y) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Radius
        arrayToReturn[1] = Math.atan(y / x) * (Math.PI / 180); // Theta


        return arrayToReturn;
    }


    /*
    Converts polar coordinates to cartesian
     0 = x
     1 = y
    */
    public double[] polarToCartesian(double r, double theta) {
        double[] arrayToReturn = new double[2];
        arrayToReturn[0] = r * Math.cos(theta); // X
        arrayToReturn[1] = r * Math.sin(theta); // Y

        return arrayToReturn;
    }

}
