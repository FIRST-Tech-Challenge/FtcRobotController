/*
 * Copyright (c) 2020 OpenFTC Team
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

package org.openftc.easyopencv;

import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.SwitchableCameraName;

class OpenCvSwitchableWebcamImpl extends OpenCvWebcamImpl implements OpenCvSwitchableWebcam
{
    public OpenCvSwitchableWebcamImpl(SwitchableCameraName cameraName)
    {
        super(cameraName);
    }

    public OpenCvSwitchableWebcamImpl(SwitchableCameraName cameraName, int containerLayoutId)
    {
        super(cameraName, containerLayoutId);
    }

    @Override
    public synchronized void setActiveCamera(WebcamName cameraName)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("setActiveCamera() called, but camera device is not opened!");
        }

        ((SwitchableCamera)camera).setActiveCamera(cameraName);
    }

    @Override
    public synchronized WebcamName getActiveCamera()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("getActiveCamera() called, but camera device is not opened!");
        }

        return (WebcamName) ((SwitchableCamera)camera).getActiveCamera();
    }

    @Override
    public synchronized WebcamName[] getMembers()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("getActiveCamera() called, but camera device is not opened!");
        }

        return (WebcamName[]) ((SwitchableCamera)camera).getMembers();
    }
}
