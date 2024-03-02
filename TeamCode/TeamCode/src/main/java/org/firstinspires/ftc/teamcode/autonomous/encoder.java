package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Encoder Test ")
public class encoder extends OpMode {
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    double ticks = 537.7;
    double newTarget;
    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf_drive");
        rf = hardwareMap.get(DcMotor.class, "rf_drive");
        lb = hardwareMap.get(DcMotor.class, "lb_drive");
        rb = hardwareMap.get(DcMotor.class, "rb_drive");

        telemetry.addData("Hardware: ", "Initialized");
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            encoder(2);
        }
        telemetry.addData("Motor Ticks: ", lf.getCurrentPosition());
        telemetry.addData("Motor Ticks: ", rf.getCurrentPosition());
        telemetry.addData("Motor Ticks: ", lb.getCurrentPosition());
        telemetry.addData("Motor Ticks: ", rb.getCurrentPosition());
        if(gamepad1.b){
            tracker();
        }
        telemetry.update();

    }
    public void encoder(int turnage){
        newTarget = ticks/turnage;

        lf.setTargetPosition((int)newTarget);
        rf.setTargetPosition((int)newTarget);
        lb.setTargetPosition((int)newTarget);
        rb.setTargetPosition((int)newTarget);

        lf.setPower(0.3);
        rf.setPower(0.3);
        lb.setPower(0.3);
        rb.setPower(0.3);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
    public void tracker(){
        lf.setTargetPosition(0);
        rf.setTargetPosition(0);
        lb.setTargetPosition(0);
        rb.setTargetPosition(0);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}