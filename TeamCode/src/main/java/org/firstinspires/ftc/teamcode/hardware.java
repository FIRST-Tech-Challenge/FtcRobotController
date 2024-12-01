package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class hardware {
    // Arm components
    public DcMotor mantis; //EXP PRT 0
    public DcMotor lift; //EXP PRT 1
    public DcMotor hopper; //EXP PRT 2

    // Servos
    public CRServo wrist; //EXP PRT 0
    public CRServo bottomGrabber; //EXP PRT 1
    public CRServo topGrabber; //EXP PRT 2
    public Servo door; //EXP PRT 3

    // Wheel components
    public DcMotor frontLeft; //CTRL PRT 0
    public DcMotor frontRight; //CTRL PRT 1
    public DcMotor backLeft; //CTRL PRT 2
    public DcMotor backRight; //CTRL PRT 3

    public ColorSensor colorSensor;

    public DistanceSensor distanceSensorLeft;
    public DistanceSensor distanceSensorRight;
    public DistanceSensor distanceSensorBack;

    public void checkMotorInit(){

    }

    public final String[] puns = {
            "A robot didn’t want to have his photo taken. When he was asked why, he replied: Because I’m a photo-resistor!",
            "A robot gets arrested. He’s charged with battery.",
            "A robot man walks into a robot restaurant. A robot waiter approaches and asks him for his robot order.",
            "A robot musician’s collection of instruments will never be complete. They can never get any organs.",
            "A robot walks into a bar and says he needs to loosen up. So the bartender serves him a screwdriver.",
            "A robot walks into a bar. The bartender asks, 'What’ll ya have?' The robot says, 'Well, it’s been a long day and I need to loosen up. How about a screwdriver?'",
            "Did you hear about the writing robot who combined all the books ever written into one big novel? It’s a long story.",
            "Does R2D2 have any brothers? No. Only transisters.",
            "Hey, did you hear the story about the headless robot? According to reports, he completely lost his mind!",
            "How are A.I. blogs similar to philosophy majors? That’s easy… they’re both always trying to explain what ‘deep learning’ is!",
            "How did the robot get across the river? In a ro-boat.",
            "How did the robot’s teacher mark his book? With robo-ticks.",
            "How do you know when you’re in love with a robot? You feel a little spark.",
            "How do you reboot a robot? You kick it in its robutt.",
            "How do you use a remote control to calm down a robot dog? Press the paws button.",
            "How long is the robot alphabet? There are just two numbers – 0 and 1!",
            "How many robots does it take to screw in a light bulb? Three — one to hold the bulb, and two to turn the ladder!",
            "I bought one of those early 2000s robot dogs but have nowhere to charge it; I can’t find a place to pug it in.",
            "I finally fulfilled my dream to become a half-cyborg! It did cost me an arm and a leg, though.",
            "I got a new robot dog last week. Its name is Dogmatic.",
            "I invented a surgical robot. So far it only operates on batteries.",
            "I just got a wireless robot the other day. You could say that our relationship comes with no strings attached.",
            "I was bored, so I made a robot that distributes herbs. It helped pass the thyme.",
            "I’m not saying all factory workers are robots… All I’m saying is when they get to work they’ve returned to their factory setting.",
            "I’m starting to make a robot that has really high words per minute count. He’s a pro-to-type.",
            "If a Norwegian robot analyzed a bird, then it… Scandinavian.",
            "In a robot-only disco, one of the dancers suddenly shuts down. The doctor arrives and after a quick inspection he calms the crowd: 'Don’t worry, he just got disco-nnected.'",
            "Inventor: 'Hey, will you give me a hand?' Robot: Detaches hand, hands it to the inventor.",
            "Judge: 'So, Mr. Robot. Your neighbor accused you of stealing their electricity to power yourself. How do you plea?' Robot, the defendant: 'Guilty as charged.'",
            "My wife told me robots don’t wash themselves. So I put one in the bath and said 'That’ll shower.'",
            "Scientists have discovered a planet populated entirely by robots. They call it Mars.",
            "A robot orders a robot steak. The robot waiter asks them how they want their robot steak prepared. The robot replies, 'Weld on.'",
            "There was a giant, steel, robot who had one job, protect the city. One day when it was raining some of the screws got rusty and fell off causing one of the legs to fall off entirely. When the leg fell off it crushed the city that it was meant to protect. Oh, the iron knee!"
    };
}
