package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.MegalodogChassis;

public class Megalodog {
    public int top_bucket=2000;
    public int low_bucket=1000;
    public int ArmLinearSlide;
    public int intakeExtension;
    public int Lift;
    public int grabSpeiceman;
    public int hangSpeciceman;
    public int ReturnLift;
    public void runintake(int direction,int howLong,int wait){
        //turn servo
    }
    public void RealeaseIntoBucket(int wait){
        //spin servo opposite
    }
    public void Returnlift(int wait){
        //turn motor
    }
    public void GrabandLift(int height, int wait){
        //turn servo and raise lift
    }

    public int ArmGround;
    public int SampleLow;
    public int SampleHigh;

    public int LinearSlideFar;
    public int LinearSlideHome;

    public int SpecimenHigh;
    public int SpecimenLow;

    public int SpecimenGripperClosed;
    public int SpecimenGripperOpen;
    //------------------------------------------------------------------------
    public void MoveSlideAndScoop (int distanceMM,int wait){};

    public void RaiseLift (int hightMM, int wait){};

    public void EmptyLift (int wait){};

    public void LetGoOfSpecimen(int wait){};

    public void CheckSampleColor (){};

    private int topBucket = 2000;
    private int lowBucket = 1000;
    private int highBar = 1500;
    private int lowBar = 500;
    private double Gripper = 105.8;
    private int highChamber = 1200;
    private int lowChamber = 255;
    private int deliveryBox = 105;
    private Servo Intake;


    public void TurnIntakeOn () {
//It turns the continues servo forward

    }
    public void TurnIntakeOff () {
//It turns the continues servo off

    }
    public void GrabSpeicen (int waitime) {
//The servo rotates forward

    }
    public void HookAndLetGo (int waitime) {
// It pushes the sepiecem down and then lets go of it

    }
}

