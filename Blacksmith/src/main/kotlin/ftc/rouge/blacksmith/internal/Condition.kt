package ftc.rouge.blacksmith.internal

/**
 * Condition is a functional interface that defines a boolean-valued function.
 * It contains a single abstract method named `evaluate` that takes no arguments and returns a `Boolean`.
 *
 * This interface also provides an `invoke` operator function that allows the instance of the interface to be
 * called like a regular function, and returns the result of `evaluate()`.
 *
 * Example usage in Kotlin:
 *
 *     // Define a condition that checks whether a variable has been initialized
 *     val variableIsInitialized = Condition { variable != null }
 *
 *     // Use the condition with `evaluate`
 *     val result1 = variableIsInitialized.evaluate()  // result1 == false
 *
 *     // Use the condition with `invoke`
 *     val result2 = variableIsInitialized()  // result2 == false
 *
 * Example usage in Java:
 * ```java
 *     // Define a condition that checks whether a variable has been initialized
 *     Condition variableIsInitialized = () -> variable != null;
 *
 *     // Use the condition with `evaluate`
 *     boolean result1 = variableIsInitialized.evaluate();  // result1 == false
 *
 *     // Use the condition with `invoke`
 *     boolean result2 = variableIsInitialized.invoke();  // result2 == false
 * ```
 *
 * @author KG
 *
 * @see ftc.rouge.blacksmith.Scheduler [Scheduler]
 * @see ftc.rouge.blacksmith.listeners.Listener [Listener]
 */
fun interface Condition {
    /**
     * Evaluates the condition and returns a boolean value.
     *
     * @return the result of evaluating the condition.
     */
    fun evaluate(): Boolean

    /**
     * Operator function that allows the instance of the interface to be called like a regular function.
     *
     * @return the result of `evaluate()`.
     */
    operator fun invoke(): Boolean {
        return evaluate()
    }
}
