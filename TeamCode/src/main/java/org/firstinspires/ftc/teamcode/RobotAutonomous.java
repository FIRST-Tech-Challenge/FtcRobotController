package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "SciRavens-Autonomous")
public class RobotAutonomous extends LinearOpMode {
    public Robot robot;
    public DriveTrain DT;
    public DroneLauncher DL;
    public Slider slider;
    public Arm arm;
    public Claw claw;

    public Hang hang;
    public AprilTag tag;
    public TgeDetection tge;
    String curAlliance = "red";
    public int zone = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        DT = new DriveTrain(robot, gamepad1);
        DL = new DroneLauncher(robot, gamepad1);
        slider = new Slider(robot, gamepad2);
        arm = new Arm(robot, gamepad2);
        claw = new Claw(robot, gamepad2);
        hang = new Hang(robot, gamepad2);
        tag = new AprilTag(robot);
        tge = new TgeDetection(robot.webcam);

        waitForStart();
        if(opModeIsActive()) {
            while (zone <= 0 || zone > 3) {
                if (gamepad1.x){
                    curAlliance = "blue";
                } else if (gamepad1.b){
                    curAlliance = "red";
                }
                tge.setAlliance(curAlliance);
                telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
                telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
                telemetry.update();
                zone = tge.elementDetection(telemetry);
            }

            switch(zone) {
                case 1:
                    auton1();
                    break;
                case 2:
                    auton2();
                    break;
                case 3:
                    auton3();
                    break;
            }

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

    public void auton1() {

    }
    public void auton2() {
        DT.drive_forward(100);
        arm.arm_pixel();
        claw.open_claw_right();
        arm.arm_backdrop();
        arm.arm_fold();
        DT.strafe_right(30);
        DT.drive_forward(100);
        DT.twist_left(90);
        DT.drive_forward(200);
        DT.strafe_left(100);
        DT.drive_forward(20);
        DT.stopDrive();
    }
    public void auton3() {

    }
}

