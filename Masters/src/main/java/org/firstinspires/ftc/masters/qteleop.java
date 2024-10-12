package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.masters.components.DriveTrain;
//import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;

@Config // Enables FTC Dashboard
@TeleOp(name = "qteleop", group = "ri30h")
public class qteleop extends LinearOpMode {

    public static int floor = 0;
    public static int highC = 1000;
    public static int high = 2500;

    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap);
        Outake outake = new Outake(hardwareMap, telemetry);

        outake.close();

        waitForStart();

        while (opModeIsActive()) {

            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);

            if (gamepad1.a) {outake.forward();}
            if (gamepad1.b) {outake.backward();}

            if (gamepad1.x) {outake.open();}
            if (gamepad1.y) {outake.close();}

            if (!outake.getExtendSlide().isBusy()) {
                outake.getExtendSlide().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad1.right_stick_y != 0) {
                    outake.extendPower(-Math.pow(gamepad1.right_stick_y, 3));
                } else {
                    outake.extendStop();
                }
            }

            if (gamepad1.dpad_up){
                outake.extendPos(highC);
            }
            if (gamepad1.dpad_down){
                outake.extendPos(floor);
            }
            if (gamepad1.dpad_right){
                outake.extendPos(high);
            }

            if (gamepad1.left_bumper) {
                outake.rotateUp();
            } else if (gamepad1.right_bumper) {
                outake.rotateDown();
            } else {
                outake.rotateStop();
            }

        }
    }
}