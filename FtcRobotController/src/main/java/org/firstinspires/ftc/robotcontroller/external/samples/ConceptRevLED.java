/*
        Copyright (c) 2024 Alan Smith
        All rights reserved.
        Redistribution and use in source and binary forms, with or without modification,
        are permitted (subject to the limitations in the disclaimer below) provided that
        the following conditions are met:
        Redistributions of source code must retain the above copyright notice, this list
        of conditions and the following disclaimer.
        Redistributions in binary form must reproduce the above copyright notice, this
        list of conditions and the following disclaimer in the documentation and/or
        other materials provided with the distribution.
        Neither the name of Alan Smith nor the names of its contributors may be used to
        endorse or promote products derived from this software without specific prior
        written permission.
        NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
        LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
        THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
        ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
        FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
        DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
        SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
        TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
        THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

/*
 * This OpMode illustrates how to use the REV Digital Indicator
 *
 * This is a simple way to add the REV Digital Indicator which has a red and green LED.
 * (and as you may remember, red + green = yellow so when they are both on you can get yellow)
 *
 * Why?
 * You can use this to show information to the driver.   For example, green might be that you can
 * pick up more game elements and red would be that you already have the possession limit.
 *
 * This OpMode assumes that the REV Digital Indicator is setup as 2 Digital Channels named
 * front_led_green and front_led_red. (the green should be the lower of the 2 channels it is plugged
 * into and the red should be the higher)
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * You can buy this product here:  https://www.revrobotics.com/rev-31-2010/
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp(name = "Concept: RevLED", group = "Concept")
@Disabled
public class ConceptRevLED extends OpMode {
    LED frontLED_red;
    LED frontLED_green;

    @Override
    public void init() {
        frontLED_green = hardwareMap.get(LED.class, "front_led_green");
        frontLED_red = hardwareMap.get(LED.class, "front_led_red");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            frontLED_red.on();
        } else {
            frontLED_red.off();
        }
        if (gamepad1.b) {
            frontLED_green.on();
        } else {
            frontLED_green.off();
        }
    }
}
