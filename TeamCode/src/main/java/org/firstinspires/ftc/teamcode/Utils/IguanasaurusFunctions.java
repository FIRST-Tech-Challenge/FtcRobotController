package org.firstinspires.ftc.teamcode.Utils;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.IguanasaurusFunctions;

public class IguanasaurusFunctions {
    boolean isPressingStart = false;
    boolean isPressingBack = false;
    boolean isPressingLeftStickButton = false;
    boolean isPressingRightStickButton = false;
    boolean isPressingLeftBumper = false;
    boolean isPressingRightBumper = false;
    boolean isPressingLeftTrigger = false;
    boolean isPressingRightTrigger = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;
    boolean isPressingDpadLeft = false;
    boolean isPressingDpadRight = false;
    boolean isPressingA = false;
    boolean isPressingB = false;
    boolean isPressingX = false;
    boolean isPressingY = false;
    boolean hasSample = false;
}
