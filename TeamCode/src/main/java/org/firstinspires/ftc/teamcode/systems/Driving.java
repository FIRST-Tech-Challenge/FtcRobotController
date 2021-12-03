package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controller;

public class Driving extends System {

    private final double FortiFiveInRads = -Math.PI/4, sin45 = Math.sin(FortiFiveInRads), cos45 = Math.cos(FortiFiveInRads);
    private DcMotor flm, frm, blm, brm;
    private double x1, x2, y1, y2;

    public Driving(HardwareMap hw, Controller controller){
        super(hw, controller);
    }

    public void init(){
        flm = hw.dcMotor.get("frontLeftDrive");
        frm = hw.dcMotor.get("frontRightDrive");
        blm = hw.dcMotor.get("backLeftDrive");
        brm = hw.dcMotor.get("backRightDrive");

        flm.setDirection(DcMotorSimple.Direction.FORWARD);
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.FORWARD);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
        y1 = -controller.gamepad1.left_stick_y;
        x1 = -controller.gamepad1.right_stick_x;

        y2 = x1*sin45 + y1*cos45;
        x2 = x1*cos45 - y1*sin45;

        y2 /= 2;
        x2 /= 2;

        //power
        flm.setPower(y2);
        frm.setPower(x2);

        blm.setPower(y2);
        brm.setPower(x2);

        controller.telemetry.addData("x2", "%.2f", x2);
        controller.telemetry.addData("y2", "%.2f", y2);
    }
}
