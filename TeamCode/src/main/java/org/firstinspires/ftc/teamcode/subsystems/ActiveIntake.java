package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.AxonAbsolutePositionEncoder;

public class ActiveIntake {

    //Adjustable constraints/constants
    private double MAX_POWER = 1;

    //Internal Variables
    private CRServo intake;
    private boolean isRunning;

    private double lastAngle;
    private long lastTime;

    private AxonAbsolutePositionEncoder encoder;

    private boolean sampleCaptured;

    /**
     * Quick constructor used to create an Active Intake Subsystem Class
     * @param hw [HardwareMap] Hardware map used to initialize servos.
     */
    public ActiveIntake(HardwareMap hw){
        this(hw, "intake");
        isRunning=false;
        encoder = new AxonAbsolutePositionEncoder(hw, "encoder");
        lastAngle = encoder.getAngle();
        lastTime = System.currentTimeMillis();
        sampleCaptured = false;
    }

    /**
     * Primary constructor used to create an Active Intake Subsystem Class
     * @param hw [HardwareMap] Hardware map used to initialize servos.
     * @param name [String] Name of the intake servo set in configuration.
     */
    public ActiveIntake(HardwareMap hw, String name){
        intake = hw.get(CRServo.class, name);
        isRunning=false;
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Spins intake servo to be able to intake samples
     */
    public void activateIntake(){
        intake.setPower(MAX_POWER);
        isRunning=true;
    }

    /**
     * Spins intake servo to be able to deposit samples
     */
    public void reverseIntake(){
        intake.setPower(-MAX_POWER);
        isRunning=true;
    }

    /**
     * Turns off all power supplied to intake servo.
     */
    public void deactivateIntake(){
        intake.setPower(0);
        isRunning = false;
    }
    public boolean isRunning() {
        return isRunning;
    }

    public void checkForSample(){
        long currentTime = System.currentTimeMillis();
        //This only updates if the sample is captured every 0.2 seconds
        if (currentTime > lastTime + 200){
            sampleCaptured = capturedSample();
        }
    }

    public boolean capturedSample(){
        double currentAngle = encoder.getAngle();
        long currentTime = System.currentTimeMillis();

        //Angle velocity in degrees/seconds
        double angelVel = (currentAngle - lastAngle) / (currentTime - lastTime) * 1000;


        lastAngle = currentAngle;
        lastTime = System.currentTimeMillis();

        //If the servo is running
        //AND the angular velocity is less than 5 degrees/second
        if(isRunning && angelVel < 5){
            return true;
        }
        return false;
    }

    @Override
    public String toString(){
        return String.format(
                "Active intake Powered: %b\n" +
                "Captured Sample: %b\n",
                isRunning,
                sampleCaptured);
    }
}
