package org.firstinspires.ftc.teamcode.toolkit.background;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Background;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

public class ShotCounter extends Background {
    LinearOpMode opMode;
    UpliftRobot robot;
    DistanceSensor shooterSensor;


    public ShotCounter(UpliftRobot robot) {
        super(robot);
        this.robot = robot;
        this.opMode = robot.opMode;
        this.shooterSensor = robot.shooterSensor;
    }

    @Override
    public void loop() {
        if(shooterSensor.getDistance(DistanceUnit.CM) < 2.0 && robot.flickingState == UpliftRobot.FlickingState.FLICKING) {
            while(shooterSensor.getDistance(DistanceUnit.CM) < 2.0 && robot.flickingState == UpliftRobot.FlickingState.FLICKING){
                Utils.sleep(50);
            }
            robot.shotCount += 1;
        }
    }
}
