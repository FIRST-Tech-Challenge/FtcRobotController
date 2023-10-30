package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "Concept")


public class AutoTeach extends LinearOpMode {
    private LinearOpMode newOp;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    public void init(LinearOpMode opMode) throws InterruptedException {
        newOp = opMode;
        // MOTOR INITIALIZATION
        fl = opMode.hardwareMap.dcMotor.get("fl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        br = opMode.hardwareMap.dcMotor.get("br");

        // Normally, should be two reversed, two forward. Reasoning as to why this works is unknown at the time.
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();

    }

    public ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        init(this);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newOp.idle();

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        newOp.idle();

        waitForStart();

        fl.setPower(1);
        fr.setPower(1);
        bl.setPower(1);
        br.setPower(1);

        sleep(1000);

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }
}
