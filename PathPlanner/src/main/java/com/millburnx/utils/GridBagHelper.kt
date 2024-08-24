package com.millburnx.utils

import java.awt.GridBagConstraints
import javax.swing.JComponent
import javax.swing.JPanel

/**
 * A basic helper class for adding components to a JPanel using GridBagLayout. The class automatically increments the current column and row as components are added.
 */
class GridBagHelper(val parent: JPanel, val columns: Int) {
    private var row = 0
    private var column = 0
    val constraints = GridBagConstraints()

    /**
     * Adds a component to the parent JPanel. Components are added left-to-right, top-to-bottom.
     * @param component The component to add.
     * @param gridWidth The number of columns the component should span.
     * @param gridHeight The number of rows the component should span.
     */
    fun add(component: JComponent, gridWidth: Int = 1, gridHeight: Int = 1) {
        constraints.gridx = column
        constraints.gridy = row
        constraints.gridwidth = gridWidth
        constraints.gridheight = gridHeight
        constraints.fill = GridBagConstraints.NONE
        constraints.anchor = GridBagConstraints.BASELINE_LEADING
        parent.add(component, constraints)

        column += gridWidth
        if (column >= columns) {
            column = 0
            row++
        }
    }

    /**
     * Adds a list of components to the parent JPanel.
     * @param components The list of components to add.
     * @see add
     */
    fun addAll(components: List<JComponent>) {
        components.forEach { add(it) }
    }
}