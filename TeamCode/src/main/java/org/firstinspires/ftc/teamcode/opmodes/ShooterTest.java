package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Shooter;

@TeleOp
public class ShooterTest extends LinearOpMode {

    public static double POWER_SHOOTER = 1;

    boolean stateA = false;
    boolean stateB = false;

    boolean motorState = false;
    boolean mState = false;


    @Override
    public void runOpMode() {
        Shooter sh = new Shooter(this);

        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.a && !stateA && !motorState) {
                sh.setPower(POWER_SHOOTER);
                motorState = true;
            } else if (gamepad1.a && !stateA && motorState) {
                sh.setPower(0);
                motorState = false;
            }

            if(gamepad1.b && !stateB && !mState) {
                sh.setPower(-POWER_SHOOTER);
                mState = true;
            } else if (gamepad1.b && !stateB && mState) {
                sh.setPower(0);
                mState = false;
            }



            stateA = gamepad1.a;
            stateB = gamepad1.b;
            telemetry.addData("power = ", POWER_SHOOTER);
            telemetry.update();

        }
    }
}
