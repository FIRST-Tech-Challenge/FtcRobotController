package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutomaticPivot {
    private Arm arm;
    private Wrist wrist;
    private IMU imu;

    //Assuming position change of 0.25 = 90 degrees
    //0.74
    public static double PARALLEL_OFFSET = 0.25;
    public static double PERPENDICULAR_OFFSET = PARALLEL_OFFSET - 0.3;

    public boolean isParallel = true;

    public AutomaticPivot(HardwareMap hw){
        //Initialize arm
        arm = new Arm(hw, "frontArm", "backArm");
        //Initialize wrist
        wrist = new Wrist(hw, "wrist");

        //Initialized IMU
        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));


    }

    /**
     * Sets the Servos to an initial parallel position
     * Subject to change in the future.
     */
    public void init(){
        arm.setParallel();
        wrist.setPosition(0.5 - PARALLEL_OFFSET);

    }

    /**
     * Sets mode to Perpendicular
     */
    public void setModePerpendicular(){
        isParallel = false;
    }

    /**
     * Sets mode to Parallel
     */
    public void setModeParallel(){
        isParallel = true;
    }

    /**
     * Toggles the mode from:
     * Parallel <--> Perpendicular
     */
    public void toggleMode(){
        isParallel = !isParallel;
    }

    /**
     * Updates the Wrist position relative to the
     * arm and robot's pitch (forward/backward tilt)
     */
    public void updatePos(){
        //Getting the initial states of the robot and arm
        double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        double armPos = arm.getPosition();

        //Maintain parallel Claw
        if(isParallel){
            wrist.setPosition(0.5 + (-pitch * 0.25) + (0.5-armPos) + PARALLEL_OFFSET);

        }
        //Maintain perpendicular Claw
        else{
            wrist.setPosition(0.5 + (-pitch * 0.25) + (0.5-armPos) + PERPENDICULAR_OFFSET);
        }
    }

    /**
     * Wrapper function to move the arm pivot
     * @param increment Used for [Trigger] Inputs to move the arm pivot
     */
    public void adjustArm(double increment){
        arm.adjustPosition(increment);
    }

    /**
     * Moves the wrist through tuning offset values
     * @param increment Used for [Button] Inputs to tune the offset values
     */
    public void adjustOffset(double increment){
        if(isParallel){
            PARALLEL_OFFSET = Math.max(Math.min( PARALLEL_OFFSET + 0.0005 *increment, 1) ,0);
        }else{
            PERPENDICULAR_OFFSET = Math.max(Math.min( PERPENDICULAR_OFFSET + 0.0005 *increment, 1) ,0);
        }
    }
    public void setPos(int pos)
    {
        wrist.setPosition(pos);
    }
    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("IMU pitch reading: %.2f\n" +
                "Arm Position: %f\n" +
                "Wrist Position: %f\n" +
                "Parallel Offset:%f\n" +
                "Perpendicular Offset:%f\n" +
                "Current Mode: %s",
                imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES),
                arm.getPosition(),
                wrist.getPosition(),
                PARALLEL_OFFSET,
                PERPENDICULAR_OFFSET,
                isParallel ? "Parallel" : "Perpendicular");
    }


}
