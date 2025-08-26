

//This file is a test for viper slides. Save this for testing

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DemoTeleOp")

public class ViperSlidesOp extends LinearOpMode {

    private DcMotor slideMotor;
    private final double slidePowerUp = 0.5;
    private final double slidePowerDown = -0.5;
    private int position = 0;
    private int previousPosition=0;

    private final double minSlidePosition = 0;
    private final double maxSlidePosition = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        slideMotor = hardwareMap.dcMotor.get("slide");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot robot = new Robot();

        double leftstick = -gamepad1.left_stick_y;

        position += (int) leftstick * 50;
        position = Math.min(position, 2910);
        position = Math.max(position, 0);

        slideMotor.setTargetPosition(position);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (position-previousPosition > 0){
            slideMotor.setPower(leftstick);
        } else if (position-previousPosition < 0) {
            slideMotor.setPower(-leftstick);
        }

        previousPosition = position;


    }
}
