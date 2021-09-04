package org.firstinspires.ftc.teamcode;
import android.os.DropBoxManager;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;
import java.lang.reflect.Array;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
@TeleOp
// Warning and reminder: Code just adds values constantly, this needs to be fixed. There is no value rollover, and it isn't averaging. This needs to be a no. 1 priority before it gets put inside a class and merged into the main file, and should be done as soon as possible.
public class DistanceSensorTesting extends LinearOpMode {
    private DistanceSensor DistanceSensor;
    @Override
    public void runOpMode() {
        DistanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
        double AveragedArray;
        double total;
        int index = 0;
        int ArraySize = 50;
        double [] sensorArray;
        int RingCount = 0;
        sensorArray = new double[ArraySize];
        waitForStart();
        while (opModeIsActive()) {
            if (index >= (ArraySize - 1)) {
                index = 0;
            }
            else {
                index++;
            }
            sensorArray[index] = DistanceSensor.getDistance(DistanceUnit.INCH);
            total = 0;
            for (int i = 0; i <= (ArraySize - 1); i++) {
                total = total + sensorArray[i];
            }
            AveragedArray = total / sensorArray.length;
            if (AveragedArray <= 17.6) {
                RingCount = 4;
            }
            else if (AveragedArray <= 19) {
                RingCount = 1;
            }
            else if (AveragedArray <= 19.7) {
                RingCount = 0;
            }
            telemetry.addData("Average Array", AveragedArray);
            telemetry.addData("index", index);
            telemetry.addData("ArraySize", ArraySize);
            telemetry.addData("total", total);
            telemetry.addData("Ring Count", RingCount);
            telemetry.update();
        }
    }
}