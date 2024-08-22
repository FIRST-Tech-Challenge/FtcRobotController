package com.example.depthnativelib

class NativeLib {

    /**
     * A native method that is implemented by the 'depthnativelib' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String

    companion object {
        // Used to load the 'depthnativelib' library on application startup.
        init {
            System.loadLibrary("depthnativelib")
        }
    }
}

class depthAiCamera {
    external fun prepareCameraJNI(): Unit
}