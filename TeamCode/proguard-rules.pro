# Avoid touching the FTC SDK
-keep class com.qualcomm.** {*;}
-keep class org.firstinspires.** {*;}
-keep class com.google.** {*;}
-keep class com.vuforia.** {*;}
-keep class org.tensorflow.** {*;}
-keep class javax.** {*;} # this is apparently required

-dontwarn com.qualcomm.**
-dontwarn org.firstinspires.**
-dontwarn com.vuforia.**
-dontwarn com.sun.**
-dontwarn org.tensorflow.**

# Op modes
-keep public class * extends com.qualcomm.robotcore.eventloop.opmode.OpMode

# Kotlin
-dontwarn kotlin.**

# ACME libs
# this keep is actually required for serialization
-keep class com.acmerobotics.** {*;}
-dontwarn com.acmerobotics.**

# RE2
-keep class org.openftc.** {*;}
-dontwarn org.openftc.**

# Other deps
-dontwarn com.fasterxml.**
-dontwarn org.yaml.**
-dontwarn org.apache.**
-dontwarn com.google.gson.**

# Misc
-dontnote **