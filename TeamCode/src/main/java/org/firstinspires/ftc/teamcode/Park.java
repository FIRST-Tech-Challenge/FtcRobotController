package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "PowerPlayAuto", group = "pp")
public class Park extends LinearOpMode {
    Ppbot robot = new Ppbot();
    final double speedScalar = 0.8;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello");
        telemetry.update();

        waitForStart();

        robot.BLeft.setPower(speedScalar * -0.7);
        robot.BRight.setPower(speedScalar * 0.7);
        robot.FRight.setPower(speedScalar * 1);
        robot.FLeft.setPower(speedScalar * 0.85);
        robot.Take1.setPosition(0.0);

        sleep(1000);

        robot.BLeft.setPower(0);
        robot.BRight.setPower(0);
        robot.FRight.setPower(0);
        robot.FLeft.setPower(0);
    }
}