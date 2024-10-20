package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Scalar;

@TeleOp(name="TestGameElement")
public class TestGameElement extends LinearOpMode {
    public TgeDetection tge;
    public Robot robot;
    int zone = -1;
    String color = "blue";
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        tge = new TgeDetection(robot, color);
        while(tge.getZone() == -1) {
            telemetry.addData(color + " CAMERA INIT :", zone);
            telemetry.update();
            sleep(100);
        }
        zone = tge.getZone();
        telemetry.addData(color + " INIT Zone number:", zone);
        telemetry.update();
        waitForStart();
        int i = 0;
        while(!isStopRequested()) {
            if (gamepad1.a) {
                color = "blue";
                tge.setColor("blue");
            }
            if (gamepad1.b) {
                color = "red";
                tge.setColor("red");
            }
            zone = tge.getZone();
            telemetry.addData(color + " OPMODE Zone number:", zone);
            //Scalar z = tge.getZ1();
            //telemetry.addData("Z1Colr-R",z.val[0]);
            //telemetry.addData("Z1Colr-G",z.val[1]);
            //telemetry.addData("Z1Colr-B",z.val[2]);
            //z = tge.getZ2();
            //telemetry.addData("Z2Colr-R",z.val[0]);
            //telemetry.addData("Z2Colr-G",z.val[1]);
            //telemetry.addData("Z2Colr-B",z.val[2]);
            //z = tge.getZ3();
            //telemetry.addData("Z3Colr-R",z.val[0]);
            //telemetry.addData("Z3Colr-G",z.val[1]);
            //telemetry.addData("Z3Colr-B",z.val[2]);
            telemetry.addData("Attempt:", i);
            telemetry.update();
            sleep(100);
            i++;
        }
    }
}

