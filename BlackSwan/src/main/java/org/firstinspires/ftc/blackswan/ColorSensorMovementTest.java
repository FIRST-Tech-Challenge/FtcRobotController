package org.firstinspires.ftc.blackswan;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class ColorSensorMovementTest extends LinearOpMode{
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, this);
        waitForStart();

        telemetry.addData("Back Light Level: ", robot.colorSensorBack.alpha());
        telemetry.addData("Back Red: ", robot.colorSensorBack.red());
        telemetry.addData("Back Blue: ", robot.colorSensorBack.blue());
        telemetry.addData("Back Green: ", robot.colorSensorBack.green());

        telemetry.update();

        while(robot.colorSensorBack.red()<65){
            robot.frontRight.setPower(0.4);
            robot.frontLeft.setPower(0.4);
            robot.backRight.setPower(0.4);
            robot.backLeft.setPower(0.4);
        }
            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);

            robot.back(0.4,0.25);
    }
}
