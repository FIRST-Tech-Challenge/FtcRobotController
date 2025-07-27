package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "motor move", group = "Robot")
public class bruv extends OpMode{
    DcMotor motor;
    public void init(){
        motor = hardwareMap.get(DcMotor.class,"FrontLeft");
    }
    public void loop(){
        motor.setPower(gamepad1.left_stick_y);
    }
}
