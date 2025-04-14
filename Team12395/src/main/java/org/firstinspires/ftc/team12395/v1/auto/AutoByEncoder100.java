package org.firstinspires.ftc.team12395.v1.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12395.v1.RobotHardware;

@Autonomous(name =  "Auto By Encoder 100pts", group = "Robot")
@Disabled

public class AutoByEncoder100 extends LinearOpMode{
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();

        waitForStart();
        runtime.reset();


        //strafing is always 2 inches less than the inches stated in code

        //driving is always 1 inch more than said in code.

        //keeps slides retracted until they are usedpublic class OutTakeHangFirst implements Action {
        //                @Override
        //                public boolean run(@NonNull TelemetryPacket packet) {
        //                    leftOutTake.setPosition(0.67);
        //                    rightOutTake.setPosition(0.33);
        //
        //                    return false;
        //                }
        //            }
        //            public Action outTakeHangFirst(){
        //                return new OutTakeHangFirst();
        //            }
        robot.setIntakePosition(1);
        robot.setOutClawPosition(1);
        robot.setVerticalPower(0);

        robot.driveEncoder(.75,-28.2,-28.2,-28.2,-28.2);
        robot.SetSlidePosition(robot.SLIDE_HIGH_RUNG);
        robot.setHorizontalPosition(0);
        sleep(950);
        robot.SetSlidePosition(robot.SLIDE_START);
        robot.setOutClawPosition(0);
        robot.setVerticalPower(1);
        sleep(900);
        ///// score preload specimen ^^
        robot.driveEncoder(.5, 17.05,17.05,17.05,17.05);
        robot.driveEncoder(.85, 46.85,46.85,-46.85,-46.85);
        // intake facing submersivle^^
        robot.driveEncoder(.6,44.097,-44.097,-44.097,44.097);
        //robot is current facing leftmost sample
        robot.setHorizontalPosition(1);
        sleep(200);
        robot.setIntakePosition(0);
        sleep(775);
        robot.setInClawPosition(1);
        sleep(300);
        robot.setHorizontalPosition(0);
        robot.setIntakePosition(1);
        sleep(750);
        robot.setOutClawPosition(1);
        sleep(100);
        robot.setInClawPosition(0);
        sleep(500);
        robot.setVerticalPower(2);
        sleep(950);
        robot.setOutClawPosition(0);
        sleep(200);
        robot.setVerticalPower(1);
        robot.driveEncoder(.55,11.25,-11.25,-11.25,11.25);
        robot.setHorizontalPosition(1);
        sleep(200);
        robot.setIntakePosition(0);
        sleep(700);
        robot.setInClawPosition(1);
        sleep(300);
        robot.setIntakePosition(2);
        robot.driveEncoder(.8, -46.5,-46.5,46.5,46.5);
        robot.setInClawPosition(0);
        sleep(200);
        robot.setHorizontalPosition(0);
        robot.setIntakePosition(1);
        sleep(500);
        robot.driveEncoder(.75,54,-54,-54,54);

    }
}
