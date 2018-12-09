/*
 * Copyright (C) 2014 Maciej Suminski <orson@orson.net.pl>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SERIAL_H
#define SERIAL_H

/**
 * Initializes the serial port with requested baud rate.
 * @param baud_rate is the baud rate to be used with the serial port.
 * @return false in case of error.
 */
void serial_init(unsigned int baud_rate);

/**
 * Sends a single character through the serial port (non-blocking).
 * @param c is the character to be sent.
 * @return false in case of error.
 */
void serial_putc(const char c);

/**
 * Sends a null-terminated string through the serial port (non-blocking).
 * @param string is the data to be sent.
 * @return false in case of error.
 */
void serial_puts(const char *string);

/**
 * Sends a specified number of characters through the serial port (non-blocking).
 * @param string is the data to be sent.
 * @param len is number of characters to be sent.
 * @return false in case of error.
 */
void serial_write(const char *string, int len);

/**
 * Receives a single character from the serial port (non-blocking).
 * @param c is a pointer to the variable where the read character will be saved.
 * If there is nothing in the buffer, it will not modified.
 * @return true if a character was received.
 */
//int serial_getc(char *c);

#endif /* SERIAL_H */

