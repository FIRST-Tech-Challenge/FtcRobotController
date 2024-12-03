package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class IntookTest extends LinearOpMode {
    CRServo one;
    CRServo two;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        one = hardwareMap.get(CRServo.class, "one");
        two = hardwareMap.get(CRServo.class, "two");
        two.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            if (timer.seconds() <4){
            one.setPower(1);
            two.setPower(1);
        }
            else if (timer.seconds() <8){
                one.setPower(-11);
                two.setPower(-1);
            }
            else {
                timer.reset();
            }
        }

    }
}
