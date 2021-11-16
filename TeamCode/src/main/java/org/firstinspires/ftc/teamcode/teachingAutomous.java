package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="learning automous")
public class teachingAutomous extends LinearOpMode {
     //adds object robot of type hardware
     LeraningHardware robot = new LeraningHardware();
     private ElapsedTime runtime = new ElapsedTime();
     //declares speed
     static final double TurnSpeed = 0.2;
     static final double ForwardSpeed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);
        //wait for start
        waitForStart();
        while (opModeIsActive()) {
            robot.Front_Left.setPower(ForwardSpeed);
            robot.Front_Right.setPower(ForwardSpeed);
            robot.Back_Right.setPower(ForwardSpeed);
            robot.Back_Left.setPower(ForwardSpeed);

            sleep(5000);

            robot.Front_Left.setPower(0);
            robot.Front_Right.setPower(0);
            robot.Back_Right.setPower(0);
            robot.Back_Left.setPower(0);

            dance();
            dance();
            dance();
            dance();
            dance();


        }
    }
    public void dance(){
        robot.Front_Left.setPower(-TurnSpeed);
        robot.Front_Right.setPower(TurnSpeed);
        robot.Back_Right.setPower(TurnSpeed);
        robot.Back_Left.setPower(-TurnSpeed);

        sleep(500);

        robot.Front_Left.setPower(TurnSpeed);
        robot.Front_Right.setPower(-TurnSpeed);
        robot.Back_Right.setPower(-TurnSpeed);
        robot.Back_Left.setPower(TurnSpeed);

        sleep(500);

    }
}
