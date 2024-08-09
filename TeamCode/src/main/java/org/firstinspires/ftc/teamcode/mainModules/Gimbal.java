package org.firstinspires.ftc.teamcode.mainModules;  //place where the code is located

import static java.lang.Math.atan2;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gimbal {
    private AnalogInput potentiometer = null;
    private Servo gimbalPitch = null;
    private Servo gimbalYaw = null;
    private Servo gimbalPos = null;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public void initGimbal(HardwareMap hardwareMapPorted, Telemetry telemetryPorted) {
        hardwareMap = hardwareMapPorted;
        telemetry = telemetryPorted;

        potentiometer = hardwareMap.get(AnalogInput.class, "Analog_Port_0_CH");
        gimbalPitch = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        gimbalYaw = hardwareMap.get(Servo.class, "Servo_Port_1_CH");
        gimbalPos = hardwareMap.get(Servo.class, "Servo_Port_2_CH");
    }

    public double AnglePosition() {return (potentiometer.getVoltage() * 81.8);} // 3.3v, 270 degrees
    

    public void telemetryGimbal() {
        telemetry.addData("Potentiometer Angle", AnglePosition());
    }

    public void untuck() {
        gimbalPos.setPosition(0.71);
        gimbalPitch.setPosition(0.91);
        gimbalYaw.setPosition(0.71);
    }

    public void tuck() {
        gimbalPos.setPosition(0.9);
        gimbalPitch.setPosition(1);
        gimbalYaw.setPosition(0.9);
    }
    public void moveGimbal(boolean automatic, boolean moveManual, double manualX,
                           double manualY, double ftcPoseX, double ftcPoseY, double ftcPoseZ, boolean upDateAutomatic){

        if (!automatic && moveManual){

            gimbalPos.setPosition(0.5 + manualX / 2);
            gimbalYaw.setPosition(0.5 + manualX / 2);
            //gimbalPitch.setPosition(0.5 + manualY / 2);

        } if (automatic) {

            double yawPosition = calculateServoAngle(ftcPoseX, ftcPoseZ, potentiometer.getVoltage());
            double pitchPosition = Math.atan(ftcPoseY/ftcPoseZ);

            gimbalYaw.setPosition(yawPosition);
            gimbalPos.setPosition(yawPosition);

            if (upDateAutomatic && !Double.isNaN(pitchPosition)){
                //gimbalPitch.setPosition(pitchPosition);
            }
        }

    }

    private double calculateServoAngle (double xy, double z, double Potensiometer) {

        double angleDifference = Math.atan(z/xy);
        telemetry.addData("xy", xy);
        telemetry.addData("z", z);
        telemetry.addData("angledifference", angleDifference);
        telemetry.addData("potentiometer.getVoltage()", potentiometer.getVoltage());
        double returnAngle = (1 - potentiometer.getVoltage()/3.3) + angleDifference/(Math.PI*3);
        telemetry.addData("returnAngle", returnAngle);
        if (Double.isNaN(returnAngle)){
            return 0;
        } else {
            return returnAngle;
        }



    }

}