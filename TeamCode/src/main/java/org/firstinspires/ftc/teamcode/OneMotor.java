package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
@Deprecated
public class OneMotor extends OpMode {
    private DcMotor slide = null;
    @Override
    public void init(){
        telemetry.addData("Init", "Success");
    }
    @Override
    public void loop(){
        slide  = hardwareMap.get(DcMotor.class, "slide");
        slide.setPower(gamepad1.left_stick_y);
    }

}
