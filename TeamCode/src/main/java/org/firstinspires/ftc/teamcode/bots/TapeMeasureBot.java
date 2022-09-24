package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TapeMeasureBot extends NewIntakeBot{
    public CRServo tapeExtend = null;
    public Servo tapeSwing = null;
    public Servo tapeElevate = null;

    public TapeMeasureBot(LinearOpMode opMode) {
        super(opMode);
    }

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        tapeExtend = hwMap.get(CRServo.class, "tapeExtend");
        tapeExtend.setDirection(DcMotorSimple.Direction.FORWARD);
//        tapeSwing = hwMap.get(Servo.class, "tapeSwing");
//        tapeSwing.setPosition(0.95);
        tapeElevate = hwMap.get(Servo.class, "tapeElevate");
        tapeElevate.setPosition(0.3);
    }

//    public void setSwing(double input){
//        tapeSwing.setPosition(input);
//    }
//
//    public void controlSwing(boolean left, boolean right) {
//        if (left) {
//            tapeSwing.setPosition(tapeSwing.getPosition()-0.003);
//        } else if (right) {
//            tapeSwing.setPosition(tapeSwing.getPosition()+0.003);
//        }
//    }

    public void setElevation(double input) {
        tapeElevate.setPosition(input);
    }

    public void controlElevationTape(boolean down, boolean up) {
        if (up) {
            tapeElevate.setPosition(tapeElevate.getPosition()+0.01);
        } else if (down) {
            tapeElevate.setPosition(tapeElevate.getPosition()-0.01);
        }
    }

    public void controlCoreHex(float extend, float retract) {
        if (extend > 0){
            tapeExtend.setDirection(DcMotorSimple.Direction.FORWARD);
            tapeExtend.setPower(extend);
        } else if (retract > 0){
            tapeExtend.setDirection(DcMotorSimple.Direction.REVERSE);
            tapeExtend.setPower(retract);
        } else {
            tapeExtend.setPower(0);
        }
    }

    protected void onTick() {
//        opMode.telemetry.addData("swing: ", tapeSwing.getPosition());
//        opMode.telemetry.addData("elevation: ", tapeElevate.getPosition());
//        opMode.telemetry.update();
        super.onTick();
    }
}
