package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.Robot;

@TeleOp(name="TestDriveOpMode", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {

    public Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            //Put all gamepad logic here.
            robot.drive.setDriverInput(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            if (gamepad1.aWasPressed()){
                robot.drive.toggleMode();
            }
            if(gamepad1.b){
                // You could use readings from April Tags here to give a new known position to the pinpoint
                robot.pinpoint.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());

            robot.periodic();
            robot.writeTelemetry(telemetry);
        }

    }
}
