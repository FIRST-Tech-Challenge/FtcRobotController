package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.src.Utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.RobotVoltageSensor;

public class LinearSlide implements Runnable {

    private final DcMotor linearSlide;


    private static final double motorPower = 0.5;
    private static final double ticksPerRevolution = 145.1;
    private static final double spoolRadius = 0.55 / 2; //in inches
    private static final double inchesPerRevolution = ticksPerRevolution * 2 * Math.PI * spoolRadius;
    private static final int linearSlideAngle = 68;
    private static final double stopPower = 0.13;
    public volatile boolean threadActive = true;
    public RobotVoltageSensor voltageSensor;
    public volatile int chosenPosition;


    public static final double level_one_height = 0;

    public LinearSlide(HardwareMap hardwareMap, String dcMotorName) {
        linearSlide = hardwareMap.dcMotor.get(dcMotorName);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // TODO Use run to position with values for zero power break behavior

    public LinearSlide(HardwareMap hardwareMap, String dcMotorName, RobotVoltageSensor voltSensor) {
        linearSlide = hardwareMap.dcMotor.get(dcMotorName);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.voltageSensor = voltSensor;
    }

    public void finalize() {
        this.stop();
    }

    private void setTargetHeight(int height) {
        chosenPosition = height;
    }

    public void setTargetLevel(LinearSlide.HeightLevels level) {
        switch (level) {
            case BottomLevel:
                setTargetHeight(0);
                return;
            case TopLevel:
                setTargetHeight(1);
                return;
            case MiddleLevel:
                setTargetHeight(2);
                return;
            case GetOverObstacles:
                setTargetHeight(3);
                return;
        }
    }

    public void stop() {
        threadActive = false;
    }

    public int getEncoderCount() {
        return linearSlide.getCurrentPosition();
    }

    @Override
    public void run() {
        while (threadActive) {
            double power;// power = -(distance* C1 )^3 + (voltage * C2)
            {
                double volts = voltageSensor.getVoltage();
                int encoderTicks = linearSlide.getCurrentPosition();
                int distanceFromPos = chosenPosition - encoderTicks;
                final double C1 = 0.01; //Scales the sensitivity of the function, smaller value is lower sensativity
                final double C2 = 0.017; //The approximate power required to hold itself at the current height

                power = -(Math.pow(((distanceFromPos * C1)), 3));
                power = power + volts * C2;
                power = MiscUtills.boundNumber(power);
            }
            linearSlide.setPower(power);
        }
    }


    public static enum HeightLevels {
        BottomLevel,
        MiddleLevel,
        TopLevel,
        GetOverObstacles
    }
}
