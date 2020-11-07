package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Teleop_shooter", group="Teleop")
//@Disabled
public class Teleop_shooter extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx liftL = null;
    private DcMotorEx liftR = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        liftL = hardwareMap.get(DcMotorEx.class, "liftl");
        liftR = hardwareMap.get(DcMotorEx.class, "liftr");

        liftL.setMode(RunMode.RUN_USING_ENCODER);
        liftR.setMode(RunMode.RUN_USING_ENCODER);

        liftL.setDirection(DcMotor.Direction.FORWARD);
        liftR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double LliftPower;
            double RliftPower;
            double LiftVelocity = 1120 * 3.75;
            double LiftVelocity2 = 0;

            LliftPower = gamepad2.left_stick_y ;
            RliftPower = gamepad2.right_stick_y ;

          if (LliftPower >= 0.25){
                liftL.setVelocity(LiftVelocity);
            }
            else if (LliftPower <= -0.25){
                liftL.setVelocity(-LiftVelocity);
            }
            else {
                liftL.setVelocity(LiftVelocity2);
            }

            if (RliftPower >= 0.25){
                liftR.setVelocity(LiftVelocity);
            }
            else if (RliftPower <= -0.25){
                liftR.setVelocity(-LiftVelocity);
            }
            else {
                liftR.setVelocity(LiftVelocity2);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}