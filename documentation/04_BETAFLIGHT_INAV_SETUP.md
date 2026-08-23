# Betaflight / INAV Setup

For T.R.I.V.E.N.I to control the drone and overlay the HUD, the Flight Controller must be configured to accept MSP commands.

## 1. Enable MSP on the UART
1. Open the Betaflight or INAV Configurator.
2. Go to the **Ports** tab.
3. Locate the UART you physically wired to the Radxa (e.g., UART2).
4. Turn on the **Configuration/MSP** toggle for that UART.
5. Set the Baud Rate to `115200` (must match the `baudrate` in your `config.json`).
6. Click **Save and Reboot**.

## 2. MSP OSD (Canvas Mode)
T.R.I.V.E.N.I injects its HUD directly into your FPV video feed using MSP DisplayPort (Canvas Mode), which means you don't need a separate PC to see what the AI is locking onto.

1. In the Configurator, go to the **OSD** tab.
2. Enable OSD if it isn't already.
3. Depending on your VTX (e.g., HDZero, Walksnail, DJI), ensure MSP DisplayPort is configured correctly for your video system.
4. The Radxa will automatically send `MSP_DISPLAYPORT` draw commands (`cmd 182`) at 50Hz to draw the `[ TARGET LOCK ]` brackets on your goggles.

## 3. Remote Control Overrides
When the Radxa enters the `FOLLOWING` state, it sends `MSP_SET_RAW_RC` packets to the FC. 
- Betaflight and INAV natively accept `MSP_SET_RAW_RC` as RC input.
- **Crucial Safety Rule:** Because T.R.I.V.E.N.I overlays its commands on top of your *actual* RC receiver array, your physical ARM switch and Flight Mode switches will always work.
- If things go wrong, instantly flip your designated "Follow Switch" to the `LOW` position on your radio. The Radxa will immediately cease MSP transmission, instantly returning full manual control to your sticks.
