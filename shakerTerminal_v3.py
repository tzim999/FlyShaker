"""
Fly Shaker Serial Terminal
Version 1

Requirements:
    pip install pyserial

Run:
    python shakerTerminal_v1.py
"""

from __future__ import annotations

import sys
import time
import threading
import msvcrt
from typing import Iterable

import serial
from serial import SerialException

###############################################################################
# EDIT THIS SECTION BEFORE RUNNING
###############################################################################

PORT = "COM5"             # Serial port used by the ESP32 (change as needed)
BAUD = 115200             # Must match the ESP32 firmware

# Maximum time (seconds) to wait for serial data before continuing.
SERIAL_TIMEOUT = 0.10

# Most ESP32 boards automatically reset when the serial port opens.
# Wait for the firmware to finish booting before sending commands.
ESP_BOOT_DELAY = 2.0

# Delay (seconds) between transmitted protocol commands.
# Prevents overflowing the ESP32 serial input buffer.
SEND_DELAY = 0.15

# ---------- Fly Shaker Protocol Parameters ----------

FREQ = (1,100)            # Stimulus frequency (Hz)
AMP = (20,20)              # Stimulus amplitude (0-30 maximum)
DUTY = (50,50)            # Duty cycle (% of each cycle that is HIGH)

BURST = (3,3)             # Burst duration (s); ignored in Part 2
BSILENCE = (.1,.1)          # Silence after each burst (s); ignored in Part 2

SEQ = (300,300)           # Sequence duration (s); ignored in Part 2
SSILENCE = (0,0)          # Silence between sequences (s); ignored in Part 2

SESSION = 300             # Total experiment duration (s); ignored in Part 2

REPORT = "FULL"

# REPORT = "NONE"   Print only ERROR and STATUS messages.
# REPORT = "BRIEF"  Also print successful command acknowledgments.
# REPORT = "FULL"   Currently identical to BRIEF. Future versions will
#                   also display detailed experiment progress and results.

# ---------- Single-Key Controls ----------

KEY_START  = "s"          # Start the experiment
KEY_PAUSE  = "p"          # Pause the experiment
KEY_STOP   = "x"          # Stop the experiment
KEY_STATUS = "?"          # Request current status
KEY_QUIT   = "q"          # Quit the terminal program; Ctrl-C also quits


###############################################################################
# END OF EDITABLE SECTION
###############################################################################


class FlyShakerTerminal:
    """Simple serial terminal for configuring and controlling the Fly Shaker."""

    def __init__(self) -> None:
        self.serial_port: serial.Serial | None = None
        self.running = threading.Event()
        self.print_lock = threading.Lock()
        self.reader_thread: threading.Thread | None = None

    def print_line(self, text: str = "") -> None:
        """Print without allowing the reader thread to interleave lines."""
        with self.print_lock:
            print(text, flush=True)

    def open_serial(self) -> None:
        """Open the configured serial port."""
        try:
            self.serial_port = serial.Serial(
                port=PORT,
                baudrate=BAUD,
                timeout=SERIAL_TIMEOUT,
                write_timeout=1.0,
            )
        except SerialException as exc:
            raise RuntimeError(
                f"Could not open {PORT} at {BAUD} baud.\n"
                f"Check the COM port, close Arduino Serial Monitor, and reconnect the ESP.\n"
                f"Serial error: {exc}"
            ) from exc

        # Clear old data that may have arrived during connection/reset.
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

    def start_reader(self) -> None:
        """Start a background thread that prints every complete ESP line."""
        if self.serial_port is None:
            raise RuntimeError("Serial port is not open.")

        self.running.set()
        self.reader_thread = threading.Thread(
            target=self._reader_loop,
            name="ESP serial reader",
            daemon=True,
        )
        self.reader_thread.start()

    def _reader_loop(self) -> None:
        """Continuously read and display ESP output."""
        assert self.serial_port is not None

        while self.running.is_set():
            try:
                raw = self.serial_port.readline()
            except SerialException as exc:
                self.print_line(f"\nSERIAL ERROR: {exc}")
                self.running.clear()
                break

            if not raw:
                continue

            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if text:
                self.print_line(f"ESP > {text}")

    def send(self, command: str) -> None:
        """Send one command followed by a newline."""
        if self.serial_port is None or not self.serial_port.is_open:
            raise RuntimeError("Serial port is not open.")

        command = command.strip()
        if not command:
            return

        self.print_line(f"PC  > {command}")

        try:
            self.serial_port.write((command + "\n").encode("utf-8"))
            self.serial_port.flush()
        except SerialException as exc:
            raise RuntimeError(f"Failed to send '{command}': {exc}") from exc

    @staticmethod
    def pair_command(name: str, values: tuple[int | float, int | float]) -> str:
        """Create NAME(min,max) using the values exactly as written in Python."""
        minimum, maximum = values
        return f"{name}({minimum},{maximum})"

    def configuration_commands(self) -> Iterable[str]:
        """Return the complete startup configuration in protocol order."""
        yield self.pair_command("FREQ", FREQ)
        yield self.pair_command("AMP", AMP)
        yield self.pair_command("DUTY", DUTY)
        yield self.pair_command("BURST", BURST)
        yield self.pair_command("BSILENCE", BSILENCE)
        yield self.pair_command("SEQ", SEQ)
        yield self.pair_command("SSILENCE", SSILENCE)
        yield f"SESSION({SESSION})"
        yield f"REPORT({REPORT.upper()})"

    def send_configuration(self) -> None:
        """Transmit all configuration parameters to the ESP."""
        self.print_line()
        self.print_line("Sending Fly Shaker configuration...")
        self.print_line("-" * 60)

        for command in self.configuration_commands():
            self.send(command)
            time.sleep(SEND_DELAY)

        self.print_line("-" * 60)
        self.print_line("Configuration sent.")
        self.print_line()

    def show_controls(self) -> None:
        """Display the available single-key controls."""
        self.print_line("Keyboard controls")
        self.print_line(f"  {KEY_START}  START")
        self.print_line(f"  {KEY_PAUSE}  PAUSE")
        self.print_line(f"  {KEY_STOP}  STOP")
        self.print_line(f"  {KEY_STATUS}  STATUS")
        self.print_line(f"  {KEY_QUIT}  Quit")
        self.print_line("  Ctrl-C  Quit")
        self.print_line()

    def keyboard_loop(self) -> None:
        """Read Windows single-key commands without requiring Enter."""
        key_commands = {
            KEY_START.lower(): "START",
            KEY_PAUSE.lower(): "PAUSE",
            KEY_STOP.lower(): "STOP",
            KEY_STATUS.lower(): "STATUS",
        }

        self.print_line("Ready. Press a command key.")

        while self.running.is_set():
            if not msvcrt.kbhit():
                time.sleep(0.03)
                continue

            key = msvcrt.getwch()

            # Ignore extended-key prefixes such as arrows and function keys.
            if key in ("\x00", "\xe0"):
                msvcrt.getwch()
                continue

            key_lower = key.lower()

            if key_lower == KEY_QUIT.lower():
                self.print_line("Quit requested.")
                break

            command = key_commands.get(key_lower)
            if command:
                self.send(command)
            elif key not in ("\r", "\n"):
                self.print_line(
                    f"Unknown key '{key}'. "
                    f"Use {KEY_START}, {KEY_PAUSE}, {KEY_STOP}, "
                    f"{KEY_STATUS}, or {KEY_QUIT}."
                )

    def close(self) -> None:
        """Stop the reader and close the serial port."""
        self.running.clear()

        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def run(self) -> None:
        """Open the ESP, send configuration, and enter interactive mode."""
        self.print_line("=" * 60)
        self.print_line("Fly Shaker Serial Terminal")
        self.print_line(f"Port: {PORT}    Baud: {BAUD}")
        self.print_line("=" * 60)

        self.open_serial()
        self.print_line(f"Connected to {PORT}. Waiting for ESP boot...")
        time.sleep(ESP_BOOT_DELAY)

        assert self.serial_port is not None
        self.serial_port.reset_input_buffer()

        self.start_reader()
        self.send_configuration()
        self.show_controls()
        self.keyboard_loop()


def main() -> int:
    terminal = FlyShakerTerminal()

    try:
        terminal.run()
        return 0
    except KeyboardInterrupt:
        print("\nCtrl-C received. Closing terminal.", flush=True)
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"\nERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        terminal.close()
        print("Serial terminal closed.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())

