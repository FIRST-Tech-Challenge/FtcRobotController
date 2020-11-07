package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous_Proto2", group="Autonomous")
//@Disabled
public class Autonomous_Proto2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private DcMotorEx collector;
    private Servo sensorArm;
    private Servo pushover;
    private DcMotor spindoctor1;
    private DcMotor spindoctor2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        lf  = hardwareMap.get(DcMotor.class, "lf");
        lf  = hardwareMap.get(DcMotor.class, "LF");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        spindoctor1 = hardwareMap.get(DcMotorEx.class, "spindoctor1");
        spindoctor2 = hardwareMap.get(DcMotorEx.class, "spindoctor2");
        sensorArm = hardwareMap.get(Servo.class, "sensorArm");
        pushover = hardwareMap.get(Servo.class, "pushover");

        lf.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        collector.setDirection(DcMotor.Direction.FORWARD);
        spindoctor1.setDirection(DcMotor.Direction.FORWARD);
        spindoctor2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 30.0)) {

            spindoctor1.setPower(0);
            spindoctor2.setPower(0);
            collector.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
            sensorArm.setPosition(0);
            pushover.setPosition(0);
            sleep(1000);

        }
    }
}