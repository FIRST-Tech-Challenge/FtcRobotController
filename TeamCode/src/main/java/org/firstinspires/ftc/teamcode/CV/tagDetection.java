/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.CV;

import static org.firstinspires.ftc.teamcode.CV.CVSettings.FEET_PER_METER;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.LEFT_TAG;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.MIDDLE_TAG;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.RIGHT_TAG;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.cx;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.cy;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.fx;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.fy;
import static org.firstinspires.ftc.teamcode.CV.CVSettings.tagsize;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class tagDetection extends LinearOpMode
{
    @Override
    public void runOpMode()
    {

    }

}