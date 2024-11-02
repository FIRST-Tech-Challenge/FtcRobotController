package org.firstinspires.ftc.teamcode.auton.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.compVis;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.visionLib;

@Autonomous(name="Test")
public class blueLeft extends LinearOpMode{

    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);

        visionLib a= new visionLib();
        String loc = a.init(this, compVis.Colors.BLUE, r);
        telemetry.addData("Location", loc);
        telemetry.update();
        waitForStart();

        r.moveInches(0.4, 6);
        r.moveInches(0.5, 3.5/2, Hardware.directions.RIGHT);

        switch (loc){
            default:
                r.moveInches(0.7,28.5);
                r.waiter(750);
                r.moveInches(0.7,-11);
                r.moveInches(0.6,20, Hardware.directions.RIGHT);
                r.moveInches(0.6, 24);
                r.rotate(0.6,90, Hardware.directions.RIGHT);
                r.moveInches(0.6, -82);
                break;
            case "Left":
                r.moveInches(0.7, 21);
                r.rotate(0.6,90, Hardware.directions.LEFT);
                r.moveInches(0.6,7.5);
                r.moveInches(0.6,2, Hardware.directions.RIGHT);
                break;
            case "Right":
                r.moveInches(0.7, 19);
                r.sensorRotate(0.4,-Math.PI/2);
                r.moveInches(0.6,7.5);
                r.moveInches(0.6,2, Hardware.directions.LEFT);
                break;
        }



    }
}
