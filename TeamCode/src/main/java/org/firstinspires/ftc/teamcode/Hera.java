package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hera {
    Telemetry telemetry;
    String HERA_CAPTION = "Hera Status";
    DriveTrain driveTrain;
    LinearOpMode opMode;
    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();
    HardwareInnov8Hera hwmap;

    public Hera(Telemetry telemetry, HardwareMap hwmap, LinearOpMode opMode) {
        this.opMode = opMode;
        this.hwmap = new HardwareInnov8Hera(hwmap, opMode);
        this.telemetry = telemetry;
        driveTrain = new DriveTrain(this.telemetry, this.hwmap, this.opMode   );
        this.telemetry.addData(HERA_CAPTION, "ready to go");
        this.telemetry.update();
    }

    public void stop() {
        this.telemetry.addData(HERA_CAPTION, "free");

        this.telemetry.update();
    }

    public void teleop(Gamepad gamepad1, Gamepad gamepad2) {

        while (this.opMode.opModeIsActive()) {
            this.telemetry.addData(HERA_CAPTION, "teleop-ing");
            driveTrain.teleopUpdate(gamepad1, gamepad2);
            this.telemetry.update();
        }
    }
    

    public void forwardTurn(int count) {
//        while(true) {
//            driveTrain.goForward(10000000);
//        }
//        driveTrain.turn(-90);
//        driveTrain.goForward(30);
        double motorOneEndPos = 0;
        double motorTwoEndPos = 0;
        double motorThreeEndPos = 0;
        double motorFourEndPos = 0;
        hwmap.motorOne.setPower(0.8);
        hwmap.motorTwo.setPower(0.8);
        hwmap.motorThree.setPower(0.8);
        hwmap.motorFour.setPower(0.8);
        double motorOneStartPos = hwmap.motorOne.getCurrentPosition();
        double motorTwoStartPos = hwmap.motorTwo.getCurrentPosition();
        double motorThreeStartPos = hwmap.motorThree.getCurrentPosition();
        double motorFourStartPos = hwmap.motorFour.getCurrentPosition();
        int i = 0;
            while (i < 10 && this.opMode.opModeIsActive()) {
                motorOneEndPos = hwmap.motorOne.getCurrentPosition();
                motorTwoEndPos = hwmap.motorTwo.getCurrentPosition();
                motorThreeEndPos = hwmap.motorThree.getCurrentPosition();
                motorFourEndPos = hwmap.motorFour.getCurrentPosition();
                i ++;
            }
        double motorOneDist = motorOneEndPos-motorOneStartPos;
        double motorOneSpeed =  motorOneDist/10;
        double motorTwoDist = motorTwoEndPos-motorTwoStartPos;
        double motorTwoSpeed = motorTwoDist/10;
        double motorThreeDist = motorThreeEndPos-motorThreeStartPos;
        double motorThreeSpeed = motorThreeDist/10;
        double motorFourDist = motorFourEndPos-motorFourStartPos;
        double motorFourSpeed = motorFourDist/10;
        Log.d("Motor Speeds,", motorOneSpeed + "," + motorTwoSpeed + "," + motorThreeSpeed + "," + motorFourSpeed);
        this.telemetry.addData("MotorOne Distance: ", motorOneDist);
        this.telemetry.addData("MotorTwo Distance: ", motorTwoDist);
        this.telemetry.addData("MotorThree Distance: ", motorThreeDist);
        this.telemetry.addData("MotorFour Distance: ", motorFourDist);
        //this.telemetry.addData("Motor3 - Motor1", motorThreeDist-motorOneDist);
        this.telemetry.addData("Number of Iterations", count);
        this.telemetry.update();





    }


}
