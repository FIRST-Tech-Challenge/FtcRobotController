package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketArmStates.BASKET_ARM_ALLIANCE;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketArmStates.BASKET_ARM_REST;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketArmStates.BASKET_ARM_SHARED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.BasketStates.BASKET_TRANSFER;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeMotorStates.INTAKE_REVERSING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeMotorStates.INTAKING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeServoStates.INTAKE_DOWN;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeServoStates.INTAKE_FLIPPING_DOWN;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeServoStates.INTAKE_FLIPPING_UP;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.IntakeServoStates.INTAKE_UP;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.RobotStates.SEQUENCING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.RobotStates.TRANSFERRING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.SlidesStates.SLIDES_RETRACTED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.SlidesStates.SLIDES_RETRACTING;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretRotationStates.TURRET_ROTATED;
import static org.firstinspires.ftc.teamcode.Components.Turret.extendPosition;
import static org.firstinspires.ftc.teamcode.Components.Turret.rotatePosition;
import static org.firstinspires.ftc.teamcode.Components.Turret.turret_saved_positions;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.Velocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.aVelocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.angle;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.differtime;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.xVelocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.xpos;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.yVelocity;
import static org.firstinspires.ftc.teamcode.Components.EncoderChassis.ypos;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.RobotStates.SLIDES_EXTENDED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretAAStates.TURRET_RAISED;
import static org.firstinspires.ftc.teamcode.Components.StateMachine.TurretRotationStates.TURRET_STRAIGHT;
import static java.lang.Math.E;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.log;
import static java.lang.Math.pow;
import static java.lang.Math.random;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.Components.CarouselCR;
import org.firstinspires.ftc.teamcode.Components.ChassisFactory;
import org.firstinspires.ftc.teamcode.Components.EncoderChassis;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.LedColor;
import org.firstinspires.ftc.teamcode.Components.Logger;
import org.firstinspires.ftc.teamcode.Components.OpenCVMasterclass;
import org.firstinspires.ftc.teamcode.Components.StateMachine;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.VSLAMChassis;
import org.firstinspires.ftc.teamcode.Components.tseDepositor;

public class BasicRobot{
    public static Logger logger;
    public static LinearOpMode op = null;
    public BasicRobot(LinearOpMode opMode){
        op = opMode;
        logger = new Logger(opMode);

    }
    public void readGamepad(float value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
    public void readGamepad(int value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
    public void readGamepad(double value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
    public void readGamepad(boolean value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
}
