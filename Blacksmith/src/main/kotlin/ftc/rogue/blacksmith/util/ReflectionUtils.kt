package ftc.rogue.blacksmith.util

import java.lang.reflect.Field
import java.lang.reflect.InvocationTargetException
import java.lang.reflect.Method

internal fun Any.getMethod(name: String, vararg params: Class<*>): Method {
    return this::class.java.getMethod(name, *params)
}

internal fun <T> Any.invokeMethod(name: String, vararg params: Pair<Any, Class<*>>): T {
    return this.getMethod(name, *params.map { it.second }.toTypedArray()).invoke(this, *params.map { it.first }.toTypedArray()) as T
}

internal fun <T> Any.invokeMethodI(name: String, vararg params: Any): T {
    return this.invokeMethod(name, *params.map { it to it::class.java }.toTypedArray()) as T
}

internal fun <T> Any.invokeMethodRethrowing(name: String, vararg params: Pair<Any, Class<*>>): T {
    try {
        return this.invokeMethod(name, *params)
    } catch (e: InvocationTargetException) {
        throw e.targetException
    }
}

internal fun <T> Any.invokeMethodRethrowingI(name: String, vararg params: Any): T {
    try {
        return this.invokeMethodI(name, *params)
    } catch (e: InvocationTargetException) {
        throw e.targetException
    }
}

fun Class<*>.getFieldsAnnotatedWith(annotation: Class<out Annotation>): List<Field> {
    return this.declaredFields.filter { it.isAnnotationPresent(annotation) }
}
