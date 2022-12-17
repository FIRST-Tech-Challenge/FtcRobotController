package ftc.rouge.blacksmith.util

import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test

internal class RunOnceFunctionTest {
    @Test
    fun `a runOnce function should only run the block once`() {
        var count = 0
        val runOnce = runOnce {
            count++
        }

        runOnce()
        runOnce()
        runOnce()

        assertEquals(1, count)
    }
}
