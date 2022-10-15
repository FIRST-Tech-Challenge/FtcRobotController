package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware2022 {

    enum RobotState {
        HasCone,
        NoCone
    }

    //Start with no cone.
    RobotState currentState = RobotState.NoCone;

    /**
     * Constructor
     * @param m This is the HarewareMap, which is configured on the dirver stataion.
     */
    public Hardware2022(HardwareMap m) {
        hwMap = m;
    }

    public HardwareMap hwMap;

    //motors
    public DcMotor wheelFrontRight = null;
    public DcMotor wheelFrontLeft = null;
    public DcMotor wheelBackRight = null;
    public DcMotor wheelBackLeft = null;
    public DcMotor wheelStrafe = null;

    public DcMotor Slide = null;
    public DcMotor Vertical = null;

    public Servo Encoders = null;
    public Servo grabberclaw = null;
    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;

    public void createHardware() {

        wheelFrontRight = hwMap.get(DcMotor.class, "rfWheel");
        wheelFrontLeft = hwMap.get(DcMotor.class, "lfWheel");
        wheelBackRight = hwMap.get(DcMotor.class, "rrWheel");
        wheelBackLeft = hwMap.get(DcMotor.class, "lrWheel");
        wheelStrafe = hwMap.get(DcMotor.class, "wheelStrafe");

        wheelFrontRight.setDirection(DcMotor.Direction.FORWARD);
        wheelBackRight.setDirection(DcMotor.Direction.REVERSE);
        wheelBackLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        wheelFrontRight.setPower(0);
        wheelBackRight.setPower(0);
        wheelFrontLeft.setPower(0);
        wheelBackLeft.setPower(0);
        wheelStrafe.setPower(0);

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Slide = hwMap.get(DcMotor.class, "Slide");
        Vertical = hwMap.get(DcMotor.class, "Vertical");


        Slide.setDirection(DcMotor.Direction.FORWARD);
        Vertical.setDirection(DcMotor.Direction.FORWARD);

        Slide.setPower(0);
        Vertical.setPower(0);

        sensorColor = hwMap.get(ColorSensor.class, "clawdistance");
        sensorDistance = hwMap.get(DistanceSensor.class, "clawdistance");

        Encoders = hwMap.get(Servo.class, "Encoders");
        grabberclaw = hwMap.get(Servo.class, "grabberclaw");

        Encoders.setPosition(0.4);

    }


    /**
     * This method checks current state of robot.
     *
     * @return  Enumeration of robot state.
     */
    RobotState checkState(){
        //TODO Implment the logic to check here.
        return currentState;
    }


    /**
     * This method to check if there is a cone close to the claw,
     * If so, close the claw, and change current stats to has Cone.
     *
     */
    void checkAndGrabCone ( ) {

        //Only try to gram cone if in No Cone state.
        if ( currentState.equals(RobotState.NoCone)){
            //TODO:  Logic here


            currentState = RobotState.HasCone;
        }
    }

}
