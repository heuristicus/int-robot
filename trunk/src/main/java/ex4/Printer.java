/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package ex4;

import java.util.HashMap;

/**
 *
 * @author robot
 */
public class Printer {

    //keep track of the log number
    private static int logNo = 0;
    //keep track of the last message for print once methods
    private static String lastMessage = "";
    //a hachmap of all the available colours to print to terminal in
    private static HashMap<String, String> colours = new HashMap<String, String>() {

        {
            put("REDB", "\033[1;41m");
            put("REDF", "\033[31m");
            put("GREENB", "\033[1;42m");
            put("GREENF", "\033[1;32m");
            put("YELLOWB", "\033[1;43m");
            put("YELLOWF", "\033[1;33m");
            put("BLUEB", "\033[1;44m");
            put("BLUEF", "\033[1;34m");
            put("MAGENTAB", "\033[1;45m");
            put("MAGENTAF", "\033[1;35m");
            put("CYANB", "\033[1;46m");
            put("CYANF", "\033[1;36m");
            put("WHITEB", "\033[1;47m");
            put("WHITEF", "\033[1;37m");
            put("RESET", "\033[0m");
        }
    };

    /**
     * Prints message to console with a line number to distinguish between repeating lines
     * @colour the colour to print it
     * @param message The message to print
     */
    public static void println(String message, String colour) {
        lastMessage = message;
        System.out.println(String.format("(%d) - %s%s", logNo++, colours.get(colour), message));
        System.out.print("\033[0m"); //reset colour
    }

    /**
     * Prints message to console with a line number to distinguish between repeating lines
     * @param message The message to print
     */
    public static void println(String message) {
        println(message, "RESET");
    }

    /**
     * Prints message to console with a line number to distinguish between repeating lines
     * If the next message is the same then it will not print it
     * @colour the colour to print it
     * @param message The message to print
     */
    public static void printlnOnce(String message, String colour) {
        if (!message.equals(lastMessage)) {
            println(message, colour);
        }
    }

    /**
     * Prints message to console with a line number to distinguish between repeating lines
     * If the next message is the same then it will not print it
     * @param message The message to print
     */
    public static void printlnOnce(String message) {
        if (!message.equals(lastMessage)) {
            println(message);
            lastMessage = message;
        }
    }
}
