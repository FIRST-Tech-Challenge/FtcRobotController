package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "CompBot Swerve", group = "CompBot")
public class CompBot extends LinearOpMode {

    SwerveConfig swerve = new SwerveConfig(this);
    MekanismConfig mek = new MekanismConfig(this);
    Utils utils = new Utils(this);
    GraphicTelemetry graph = new GraphicTelemetry(this);



    Thread mekThread = new Thread() {
        public void run() {

            waitForStart();
            while (opModeIsActive()) {

                boolean clipCanceled = false;

                if (gamepad2.x) {
                    mek.armLength(0);
                    while (mek.slide.isBusy() && gamepad2.y) idle();

                    mek.armAngle(20);
                    while (mek.pivot.isBusy() && gamepad2.y) idle();
                }
                mek.pauseSlide();
                mek.pausePivot();
                if (gamepad2.y) clipCanceled = true;

                if (!clipCanceled) {
                    mek.armLength(10);
                }


            }
        }

    };


    /**
     * Controls for Gamepad 1:
     * Right trigger: Forwards
     * Left trigger: Reverse
     * Right stick X: Rotate
     * Left stick X Strafe
     * <p>
     * Controls for Gamepad 2:
     * Left stick y: In and out of arm
     * Right stick y: Up and down of arm
     * Left trigger: Claw intake
     * Right trigger: Claw out
     * Presets for:
     * Attaching clip to sample
     * Attaching specimen(clip + sample) to top rung
     * Presets for bucket 1 and 2
     */
    public void runOpMode() throws InterruptedException {

        swerve.initSwerve(); // Inits all the stuff related to swerve drive
        mek.initMekanism(); // Inits the mechanism stuff

        mekThread.start();


        waitForStart();
        while (opModeIsActive()) {

            // Gamepad 1
            double speedGMP1 = gamepad1.left_trigger - gamepad1.right_trigger; // Makes it so that the triggers cancel each other out if both are pulled at the same time
            double angleGMP1 = -gamepad1.right_stick_x;

            if (speedGMP1 != 0) swerve.moveStraight(gamepad1.left_stick_x, speedGMP1);
            else if (angleGMP1 != 0) swerve.rotate(angleGMP1);
            else {
                swerve.FLMotor.setPower(0);
                swerve.BLMotor.setPower(0);
                swerve.BRMotor.setPower(0);
                swerve.FRMotor.setPower(0);
            }

        }
    }
}
