I figured out exactly why it's failing!

On the Radxa Zero 3W, Pins 8 and 10 are hardcoded at the factory as the system debug console. The operating system locks them down for the bootloader at a massive 1,500,000 baud rate, which completely conflicts with the 115200 baud rate we need for the Flight Controller.

If we force the OS to give up Pins 8 and 10, there's a high risk of causing a boot loop.

The Solution: We are going to use UART7 instead, which is completely free and specifically meant for external hardware like flight controllers.

Step 1: Change Your Wiring
Move your Flight Controller UART wires down two spots:

Pin 11 (UART7 TX) ─────► FC UART RX
Pin 13 (UART7 RX) ◄───── FC UART TX
Pin 6 (GND) ◄─────► FC GND
Step 2: Run the new setup script
I've created a new script specifically to enable UART7. Press q or Esc to exit out of the rsetup menu, and then run this command:

bash
sudo bash scripts/enable_uart7.sh
Step 3: Reboot and Test
Once the script finishes, run:

bash
sudo reboot
When it boots back up, run the test script again:

bash
python scripts/test_hardware.py
It should now connect successfully on /dev/ttyS7 and you'll see the RC and OSD tests pass! Let me know when you've moved the wires and run the test.

1:17 AM
