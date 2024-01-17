package org.firstinspires.ftc.teamcode.Toros.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class PIDF_Gamepad extends LinearOpMode{
    private PIDController controller;
    
    public static double p = 0.03, i = 0, d = -0.0001;
    public static double f = -0.05;

    public static int target = -75;
    private final double ticks_in_degrees = 1440/180;
    private DcMotorEx Arm;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm = hardwareMap.get(DcMotorEx.class,"Arm");

        waitForStart();
        while (opModeIsActive()){
            controller.setPID(p,i,d);
            int armPos = Arm.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

            double power = pid + ff;

            Arm.setPower(power);

            if (gamepad2.y) {
                target = -1300;
            } else if (gamepad2.x) {
                target = -1900;
            } else if (gamepad2.b){
                target = -75;
            } else if(gamepad2.a){
                target =-3250;
            }

            if (gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0 ){
                target = (int) (target + 0 * gamepad2.left_stick_y);
            }

            telemetry.addData("Pos", armPos);
            telemetry.addData("Target", target);
            telemetry.update();

        }
    }
}
