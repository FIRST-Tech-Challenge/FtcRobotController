package org.firstinspires.ftc.teamcode.mecanum;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class Mecanum_Drive extends LinearOpMode {
    DcMotor FLMotor, FRMotor, BLMotor, BRMotor;

    @Override
    public void runOpMode() {


        initRobot();


        waitForStart();
        while (opModeIsActive()) {
            moveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        }

    }

    public void initRobot(){
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

    }


    public void moveRobot(double FBPower, double LRPower, double turnPower){
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

}
