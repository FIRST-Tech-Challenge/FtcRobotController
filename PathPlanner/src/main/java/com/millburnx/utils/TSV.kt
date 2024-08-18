package com.millburnx.utils

import java.io.File

/**
 * Utility class for reading and writing TSV files
 */
class TSV {
    companion object {
        /**
         * Reads a TSV file and returns the data as a list of string rows
         */
        fun read(file: File): List<List<String>> {
            return parse(file.readText())
        }

        /**
         * Reads a TSV file and returns the data as a list of string rows, uses buffered reader
         */
        fun bufferedRead(file: File): List<List<String>> {
            return parse(file.bufferedReader().use { it.readText() })
        }

        /**
         * Parses a TSV text and returns the data as a list of strings rows
         */
        private fun parse(tsv: String): List<List<String>> {
            val rows = tsv.split("\n").map { parseRow(it) }
            return rows.filter { row ->
                row.isNotEmpty() && row.any { it.isNotBlank() }
            }
        }

        /**
         * Parses a single row of a TSV file
         */
        private fun parseRow(tsv: String): List<String> {
            return tsv.split("\t")
        }

        /**
         * Converts a list of rows to a TSV string
         */
        private fun stringify(data: List<List<Any>>): String {
            return data.joinToString("\n") { it.joinToString("\t") }
        }

        /**
         * Writes a list of rows to a TSV file
         */
        fun write(file: File, data: List<List<String>>) {
            file.writeText(stringify(data))
        }

        /**
         * Writes a list of rows to a TSV file, uses a buffered writer
         */
        fun bufferedWrite(file: File, data: List<List<String>>) {
            file.bufferedWriter().use { it.write(stringify(data)) }
        }
    }
}