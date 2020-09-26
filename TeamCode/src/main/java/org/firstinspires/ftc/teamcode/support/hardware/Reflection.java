package org.firstinspires.ftc.teamcode.support.hardware;

// import android.support.annotation.NonNull;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * Utility class providing common reflection operations for java beans
 */
public class Reflection {

    // All methods are static, class should not be instantiated
    private Reflection() {}

    /**
     * Returns value for given property of given object
     * @param instance object to retrieve property for
     * @param property property name
     * @return property value
     */
    public static Object get(Object instance, String property)
            throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
    {
        Method method = instance.getClass().getMethod("get" + capitalize(property));
        method.setAccessible(true);
        return method.invoke(instance);
    }

    /**
     * Sets given property of given object to given value
     * @param instance object to retrieve property for
     * @param property property name
     * @param value value to set property to, cannot be null
     */
    public static void set(Object instance, String property, Object value)
            throws NoSuchMethodException, InvocationTargetException, IllegalAccessException
    {
        Method method = instance.getClass().getMethod("get" + capitalize(property));
        Class<?> valueClass = method.getReturnType();
        // convert double to int if needed
        if (valueClass.equals(Integer.TYPE)) {
            value = new Integer(((Number) value).intValue());
        }
        method = instance.getClass().getMethod("set" + capitalize(property), valueClass);
        method.setAccessible(true);
        method.invoke(instance, value);
    }

    /**
     * Determines whether given method is a getter according to JavaBeans standard
     */
    public static boolean isGetter(Method method) {
        return (method.getParameterTypes().length == 0)
                && method.getName().length() > 3
                && method.getName().startsWith("get");
    }

    /**
     * Determines whether given method is a setter according to JavaBeans standard
     */
    public static boolean isSetter(Method method) {
        return (method.getParameterTypes().length == 1)
                && method.getName().length() > 3
                && method.getName().startsWith("set");
    }

    /**
     * Determines property name for given getter or setter method according to JavaBeans standard
     */
    public static String asProperty(Method getterOrSetter) {
        if (!isGetter(getterOrSetter) && !isSetter(getterOrSetter)) {
            throw new IllegalArgumentException("Method " + getterOrSetter.getName() + " is neither getter nor setter");
        }
        String property = getterOrSetter.getName().substring(3);
        return Character.toLowerCase(property.charAt(0)) + property.substring(1);
    }

    private static String capitalize(String property) {
        return Character.toUpperCase(property.charAt(0)) + property.substring(1);
    }
}
