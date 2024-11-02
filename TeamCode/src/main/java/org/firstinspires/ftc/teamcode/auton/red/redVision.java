package org.firstinspires.ftc.teamcode.auton.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.compVis;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.visionLib;
@Autonomous(name="Red Vision")
public class redVision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumHardAuto r = new MecanumHardAuto();
        r.initRobot(this);
        visionLib a = new visionLib();

        String loc = a.init(this, compVis.Colors.RED, r);
        telemetry.addData("Location", loc);
        telemetry.update();
        waitForStart();

        r.moveInches(0.4, 6);
        r.moveInches(0.5, 3.5/2, Hardware.directions.RIGHT);
        switch (loc){

            case "Left":
                r.moveInches(0.7, 19);
                r.moveInches(0.7,7.5, Hardware.directions.LEFT);
                r.moveInches(0.6,6);
                r.moveInches(0.6,-5);
//                r.moveInches(0.6,64, Hardware.directions.LEFT);
                break;
            case "Right":
                r.moveInches(0.7, 19);
                r.moveInches(0.7,16.5, Hardware.directions.RIGHT);
                r.moveInches(0.6,6);
                r.moveInches(0.6,-5);
//                r.moveInches(0.6,88, Hardware.directions.LEFT);
                break;
            default:
                r.moveInches(0.7,28.5);
                r.waiter(750);
                r.moveInches(0.7,-11);
//                r.moveInches(0.6,20, Hardware.directions.RIGHT);
//                r.moveInches(0.6, 32);
//                r.superRotate(0.6,90, Hardware.directions.RIGHT);
//                r.moveInches(0.6, -108);
                break;
        }
    }
}
