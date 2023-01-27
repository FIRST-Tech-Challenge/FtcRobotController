package ftc.rogue.blacksmith

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.FieldProvider
import com.acmerobotics.dashboard.config.variable.BasicVariable
import com.acmerobotics.dashboard.config.variable.ConfigVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.dashboard.config.variable.VariableType
import ftc.rogue.blacksmith.annotations.ConfigKt
import io.github.classgraph.ClassGraph
import io.github.classgraph.ClassGraph.ScanResultProcessor
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.lang.reflect.Field
import java.lang.reflect.Modifier
import java.util.concurrent.Executors

fun tast() = try {
    AppUtil.getDefContext() ?: throw RuntimeException("TODO: clean everything up")

    val executor = Executors.newFixedThreadPool(1)

    val then = ScanResultProcessor { result ->
        val classes = result.getClassesWithAnnotation(ConfigKt::class.java)

        classes.forEach { classInfo ->
            val customVar = classInfo.loadClass().run(::createVariableFromClass)

            FtcDashboard.getInstance().withConfigRoot {
                it.putVariable(classInfo.name, customVar)
            }
        }

        executor.shutdown()
    }

    ClassGraph()
        .enableClassInfo()
        .enableFieldInfo()
        .enableAnnotationInfo()
        .disableJarScanning()
        .acceptPackages("org.firstinspires.ftc.teamcode*")
        .ignoreClassVisibility()
        .filterClasspathElements {
            "TeamCode" in it
        }
        .scanAsync(executor, 1, then) {}
} catch (_: ExceptionInInitializerError) {}

fun createVariableFromClass(configClass: Class<*>): CustomVariable {
    val customVariable = CustomVariable()

    for (field in configClass.declaredFields) {
        if (Modifier.isFinal(field.modifiers))
            continue

        customVariable.putVariable(field.name, createVariableFromField(field, null))
    }

    return customVariable
}

fun createVariableFromField(field: Field, parent: Any?): ConfigVariable<*> {
    val fieldClass = field.type

    return when (val type = VariableType.fromClass(fieldClass)) {
        VariableType.CUSTOM -> {
            val customVariable = CustomVariable()

            for (nestedField in fieldClass.fields) {
                if (Modifier.isFinal(field.modifiers))
                    continue

                try {
                    customVariable.putVariable(
                        nestedField.name,
                        createVariableFromField(nestedField, field[parent])
                    )
                } catch (_: IllegalAccessException) {}
            }
            customVariable
        }

        else -> BasicVariable(
            type,
            FieldProvider<Boolean>(field, parent)
        )
    }
}
