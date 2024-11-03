package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

//public class Controller {
//
//    Gamepad gamepad;
//
//    public Controller(Gamepad gamepad) {
//        this.gamepad = gamepad;
//
//    }
//
//    public boolean aPressed;
//    public boolean aPressedPrevious;
//    public boolean aPressedDelta;
//    public boolean aReleasedDelta;
//    public boolean aReleased;
//
//    public void update() {
//        aPressedDelta = false;
//
//        aPressed = gamepad.a;
//        if (aPressedPrevious) {
//            aPressedDelta = false;
//        }
//
//        if (!aPressedPrevious && aPressed) {
//            aPressedDelta = true;
//        }
//
//        if (aPressedPrevious && !aPressed) {
//            aReleasedDelta = true;
//        }
//
//
//        aPressedPrevious = aPressed;
//
//    }
//
//}
