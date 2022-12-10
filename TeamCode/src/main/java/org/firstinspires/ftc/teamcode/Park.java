package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "PowerPlayAuto", group = "pp")
public class Park extends LinearOpMode {
    final double speedScalar = 0.8;
    Ppbot robot = new Ppbot();
    private ElapsedTime  runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        waitForStart();
        robot.BLeft.setPower(speedScalar * -0.7);
        robot.BRight.setPower(speedScalar * 0.7);
        robot.FRight.setPower(speedScalar);
        robot.FLeft.setPower(speedScalar * 0.85);

        sleep(1000);
        if (opModeIsActive() && runtime.seconds() > 2) {
            robot.Take1.setPosition(0.0);
            robot.BLeft.setPower(0);
            robot.BRight.setPower(0);
            robot.FRight.setPower(0);
            robot.FLeft.setPower(0);
        }
    }
}