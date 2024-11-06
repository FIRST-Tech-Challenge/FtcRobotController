package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
@Disabled
public class MainAuto extends LinearOpMode {
    ///////////////////////////////psuedocode///////////////////////////////
    //(robot is 17 inches long)
    //move right 24 inches
    //extend linear slide up to second ladder
    //move forward 7 inches(24-17)
    //move slide down slightly (clip specimen on second ladder)
    //release claw
    //move left a few inches (for other team's auto

    ///////////////////////////////code///////////////////////////////
    private DcMotor leftBack; //Initializes Back-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightBack; //Initializes Back-Right direct current motor for the driving function of our robot, gary.
    private DcMotor leftFront; //Initializes Front-Left direct current motor for the driving function of our robot, gary.
    private DcMotor rightFront; //Initializes Front-Right direct current motor for the driving function of our robot, gary.

    @Override
    public void runOpMode() {

    }

    private void driveInches(float inches) {
        int targetPos = leftBack.getCurrentPosition() * ;
    }

}
