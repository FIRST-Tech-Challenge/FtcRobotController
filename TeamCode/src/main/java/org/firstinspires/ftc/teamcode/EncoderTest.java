package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RESET_ENCODERS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "Encoder_Test", group = "TeleOp")
public class EncoderTest extends OpMode {
    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void init() {

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_drive"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "back_left_drive"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_drive"));



        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);





    }

    @Override
    public void loop() {
        telemetry.addData("LeftEncoder:", leftEncoder.getCurrentPosition());
        telemetry.addData("RightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("BackEncoder", frontEncoder.getCurrentPosition());
        telemetry.update();


    }
}
