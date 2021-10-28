package org.firstinspires.ftc.teamcode;
// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Generic_Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turn_Table;

@TeleOp(name="FTC_14133_2022", group="Iterative Opmode")


//My favorite shape is a nonagon
//I like to ride dirt bikes RS

//ToDo: Add light subsystem for Blinkin' which will run primarily with while loops likely

public class  FTC_14133_2022 extends OpMode {
 public Drivetrain drivetrain=null;
 public Intake Intake=null;
 public Turn_Table Turn_Table=null;
 public Generic_Lift Generic_Lift=null;

 public void init() {
     drivetrain = new Drivetrain(hardwareMap);
     Intake = new Intake(hardwareMap);
     Turn_Table = new Turn_Table(hardwareMap);
     Generic_Lift = new Generic_Lift(hardwareMap);
 }

 public void init_loop() {

 }

 public void start() {
 }

    public double tbegin = getRuntime();

 public void loop() {

    drivetrain.Update(gamepad1);
    Generic_Lift.Update(gamepad2);

    // getArmPosition here so that we can pass the minimal amount of data
    Intake.Update(gamepad2, Generic_Lift.getArmPosition());
    Turn_Table.Update(gamepad2);



}
}
