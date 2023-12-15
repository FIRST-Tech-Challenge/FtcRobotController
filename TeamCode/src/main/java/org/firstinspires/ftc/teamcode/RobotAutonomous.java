package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "SciRavens-Autonomous")
public class RobotAutonomous extends LinearOpMode {
    public Robot robot;
    public DriveTrain DT;
    public DroneLauncher DL;
    public Slider slider;
    public Arm arm;
    public Claw claw;

    public Hang hang;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        DT = new DriveTrain(robot, gamepad1);
        DL = new DroneLauncher(robot, gamepad1);
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot, gamepad2);
        hang = new Hang(robot, gamepad2);

        waitForStart();
        if(opModeIsActive()) {
            /*
            DT.drive_forward(100);
            sleep(5000);
            DT.drive_reverse(100);
            sleep(5000);
            DT.strafe_left(100);
            sleep(5000);
            DT.strafe_right(100);
            sleep(5000);
            */
            DT.twist_left(100);
            sleep(5000);
            /*
            DT.twist_right(200);
            sleep(5000);

             */
            /*
            /*
            DT.timed_drive(-0.3, 0, 0, 1.0, 3);
            telemetry.addData("FirstStep:", 2.25);
            telemetry.update();
            //wait(2);
            DT.timed_drive(-0.3, 0, 0, 1.0, 0.5);
            telemetry.addData("SecondStep:", 0.5);
            telemetry.update();
            DT.timed_drive(0.0,  0.8, 0, 1.0, 10.0);
            telemetry.addData("ThirdStep", 0.5);
             */

        }
    }
}
