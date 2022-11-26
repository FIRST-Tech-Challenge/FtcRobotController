package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.LEDStrip;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
@Disabled
@Autonomous (name = "LEDTest")

public class LEDTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        PwPRobot pwprobot = new PwPRobot(this, false);
        LEDStrip led = new LEDStrip();
led.init();
        waitForStart();

        while (opModeIsActive()) {
            led.loop();

//            //basic colors
//
//            led.red();
//
//            sleep(5000);
//
//            led.blue();
//
//            sleep(5000);
//
//            led.orange();
//
//            sleep(5000);
//
//            led.yellow();
//
//            sleep(5000);
//
//            led.gold();
//
//            sleep(5000);
//
//            led.white();
//
//            sleep(5000);
//
//            led.gray();
//
//            sleep(5000);
//
//            led.pink();
//
//            sleep(5000);
//
//            led.aqua();
//
//            sleep(5000);
//
//            led.green();
//
//            sleep(5000);
//
//            led.black();
//
//            sleep(5000);
//
//            led.confetti();
//
//            sleep(5000);
//
//            led.lime();
//
//            sleep(5000);
//
//            led.violet();
//
//            sleep(5000);
//
//
//
//
//
//            //compound colors
//
//            led.bluegreen();
//
//            sleep(5000);
//
//            led.blueviolet();
//
//            sleep(5000);
//
//            led.lawngreen();
//
//            sleep(5000);
//
//            led.redorange();
//
//            sleep(5000);
//
//            led.skyblue();
//
//            sleep(5000);
//
//
//
//            //dark
//
//            led.darkblue();
//
//            sleep(5000);
//
//            led.darkgray();
//
//            sleep(5000);
//
//            led.darkgreen();
//
//            sleep(5000);
//
//            led.darkred();
//
//            sleep(5000);
//
//
//
//
//
//            //breath
//
//            led.breathblue();
//
//            sleep(5000);
//
//            led.breathgray();
//
//            sleep(5000);
//
//            led.breathred();
//
//            sleep(5000);
//
//
//
//
//
//            //bpm
//
//            led.bpmforest();
//
//            sleep(5000);
//
//            led.bpmlava();
//
//            sleep(5000);
//
//            led.bpmocean();
//
//            sleep(5000);
//
//            led.bpmparty();
//
//            sleep(5000);
//
//            led.bpmrainbow();
//
//            sleep(5000);
//
//
//
//
//
//            //waves
//
//            led.wavesforest();
//
//            sleep(5000);
//
//            led.waveslava();
//
//            sleep(5000);
//
//            led.wavesocean();
//
//            sleep(5000);
//
//            led.wavesparty();
//
//            sleep(5000);
//
//            led.wavesrainbow();
//
//            sleep(5000);
//
//
//
//
//
//            //cp
//
//            led.cpbpm();
//
//            sleep(5000);
//
//
//
//
//
//            //cp1
//
//            led.cp1breathfast();
//
//            sleep(5000);
//
//            led.cp1breathslow();
//
//            sleep(5000);
//
//            led.cp1endtoendblacktoblack();
//
//            sleep(5000);
//
//            led.cp1heartbeatfast();
//
//            sleep(5000);
//
//            led.cp1heartbeatmedium();
//
//            sleep(5000);
//
//            led.cp1hearbeatslow();
//
//            sleep(5000);
//
//            led.cp1larsonscanner();
//
//            sleep(5000);
//
//            led.cp1lightchase();
//
//            sleep(5000);
//
//            led.cp1shot();
//
//            sleep(5000);
//
//            led.cp1strobe();
//
//            sleep(5000);
//
//
//
//
//
//            //cp12
//
//            led.cp12colorgradient();
//
//            sleep(5000);
//
//            led.cp12colorwaves();
//
//            sleep(5000);
//
//            led.cp12endtoendblend();
//
//            sleep(5000);
//
//            led.cp12sinelon();
//
//            sleep(5000);
//
//            led.cp12endtoendblend1to2();
//
//            sleep(5000);
//
//            led.cp12noblending();
//
//            sleep(5000);
//
//            led.cp12sparkle1on2();
//
//            sleep(5000);
//
//            led.cp12sparkle2on1();
//
//            sleep(5000);
//
//            led.cp12twinkles();
//
//            sleep(5000);
//
//
//
//
//
//            //cp2
//
//            led.cp2breathfast();
//
//            sleep(5000);
//
//            led.cp2breathslow();
//
//            sleep(5000);
//
//            led.cp2endtoendblendtoblack();
//
//            sleep(5000);
//
//            led.cp2heartbeatfast();
//
//            sleep(5000);
//
//            led.cp2hearbeatmedium();
//
//            sleep(5000);
//
//            led.cp2heartbeatslow();
//
//            sleep(5000);
//
//            led.cp2larsonscanner();
//
//            sleep(5000);
//
//            led.cp2lightchase();
//
//            sleep(5000);
//
//            led.cp2();
//
//            sleep(5000);
//
//            led.cp2strobe();
//
//            sleep(5000);
//
//
//
//
//
//            //fire
//
//            led.fire();
//
//            sleep(5000);
//
//            led.firelarge();
//
//            sleep(5000);
//
//            led.firemedium();
//
//            sleep(5000);
//
//
//
//
//
//            //heartbeat
//
//            led.heartbeatblue();
//
//            sleep(5000);
//
//            led.heartbeatgray();
//
//            sleep(5000);
//
//            led.heartbeatred();
//
//            sleep(5000);
//
//            led.heartbeatwhite();
//
//            sleep(5000);
//
//
//
//
//
//            //larson scanner
//
//            led.larsonscannergray();
//
//            sleep(5000);
//
//            led.larsonscannerred();
//
//            sleep(5000);
//
//
//
//
//
//            //light chase
//
//            led.lightchaseblue();
//
//            sleep(5000);
//
//            led.lightchasegray();
//
//            sleep(5000);
//
//            led.lightchasered();
//
//            sleep(5000);
//
//
//
//
//
//            //rainbow
//
//            led.rainbow();
//
//            sleep(5000);
//
//            led.rainbowforest();
//
//            sleep(5000);
//
//            led.rainbowrainbow();
//
//            sleep(5000);
//
//            led.rainbowlava();
//
//            sleep(5000);
//
//            led.rainbowocean();
//
//            sleep(5000);
//
//            led.rainbowparty();
//
//            sleep(5000);
//
//            led.rainbowwithglitter();
//
//            sleep(5000);
//
//
//
//
//
//            //shot
//
//            led.shotblue();
//
//            sleep(5000);
//
//            led.shotred();
//
//            sleep(5000);
//
//            led.shotwhite();
//
//            sleep(5000);
//
//
//
//
//
//            //sinelon
//
//            led.sinelonforest();
//
//            sleep(5000);
//
//            led.sinelonlava();
//
//            sleep(5000);
//
//            led.sinelonocean();
//
//            sleep(5000);
//
//            led.sinelonparty();
//
//            sleep(5000);
//
//            led.sinelonrainbow();
//
//            sleep(5000);
//
//
//
//
//
//            //strobe
//
//            led.strobeblue();
//
//            sleep(5000);
//
//            led.strobegold();
//
//            sleep(5000);
//
//            led.strobered();
//
//            sleep(5000);
//
//            led.strobewhite();
//
//            sleep(5000);
//
//
//
//
//
//            //twinkles
//
//            led.twinklesforest();
//
//            sleep(5000);
//
//            led.twinkleslava();
//
//            sleep(5000);
//
//            led.twinklesocean();
//
//            sleep(5000);
//
//            led.twinklesparty();
//
//            sleep(5000);
//
//            led.twinklesrainbow();
//
//            sleep(5000);
        }
    }
}
