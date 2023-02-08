package ftc.rogue.blacksmith.internal

import android.content.Context
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ClasspathScanner
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import ftc.rogue.blacksmith.annotations.ConfigKt
import java.util.concurrent.Executors

// TODO: literally everything
// all of this needs to be cleaned up, tested, and fixed lol
// also should prob make my own classpath scanner

private var configKtClassesAdded = false

//@OnCreate
fun ftcDashboardStartedListener(context: Context) {
    if (configKtClassesAdded)
        return

    val executor = Executors.newFixedThreadPool(1)

    executor.submit {
        while (FtcDashboard.getInstance() == null) {
            Thread.sleep(200)
        }

        tryConfigKtSetup(
            setOf(
                "java",
                "android",
                "com.sun",
                "com.vuforia",
                "com.google",
                "kotlin",
            )
        )

        executor.shutdown()
    }
}

fun tryConfigKtSetup(packageIgnorePrefixes: Set<String>) =
    try {
        val scanner = ClasspathScanner(object : ClasspathScanner.Callback {
            override fun shouldProcessClass(className: String): Boolean {
                return packageIgnorePrefixes.any { className.startsWith(it) }
            }

            override fun processClass(configClass: Class<*>) {
                if (!configClass.isAnnotationPresent(ConfigKt::class.java))
                    return

                val altName = configClass.getAnnotation(ConfigKt::class.java)!!.value

                val name = altName.ifEmpty { configClass.simpleName }

                FtcDashboard.getInstance()?.withConfigRoot {
                    it.putVariable(name, ReflectionConfig.createVariableFromClass(configClass))
                }
            }
        })

        scanner.scanClasspath()
    } catch (_: ExceptionInInitializerError) {}

//fun tryConfigKtSetup(executor: ExecutorService) = try {
//    val time = ElapsedTime()
//
//    val then = ScanResultProcessor { result ->
//        val classes = result.getClassesWithAnnotation(ConfigKt::class.java)
//
//        classes.forEach { classInfo ->
//            val customVar = classInfo.loadClass().run(::createVariableFromClass)
//
//            println(time.milliseconds())
//            println(classInfo.name)
//
////            FtcDashboard.getInstance()?.withConfigRoot {
////                it.putVariable(classInfo.name, customVar)
////            }
//        }
//
//        executor.shutdown()
//    }
//
//    time.reset()
//
//    ClassGraph()
//        .enableClassInfo()
//        .enableFieldInfo()
//        .enableAnnotationInfo()
//        .disableJarScanning()
//        .acceptPackages("org.firstinspires.ftc.teamcode*")
//        .ignoreClassVisibility()
//        .filterClasspathElements {
//            "TeamCode" in it
//        }
//        .scanAsync(executor, 1, then) {}
//} catch (_: ExceptionInInitializerError) {
//}
