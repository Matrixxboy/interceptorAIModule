# T.R.I.V.E.N.I — Field Operations Guide

This guide explains how to operate the AI module in the field using your RadioMaster transmitter and FPV Goggles.

## 1. RadioMaster Switch Setup
The AI relies on two switches on your radio. By default, it expects these to be mapped to **CH6** and **CH7**. You must configure two switches on your RadioMaster to output to these channels on the Receiver/Mixes tab in EdgeTX.

*   **CH7 (Target Lock Switch)**
    *   `LOW` (< 1700): Idle. The AI scans the camera feed but does nothing.
    *   `HIGH` (> 1700): **LOCK**. The AI instantly grabs whatever object is directly in the center of the camera frame and locks onto it. 
*   **CH6 (Follow / Engage Switch)**
    *   `LOW` (< 1700): Manual Control. You have full control of the drone.
    *   `HIGH` (> 1700): **ENGAGE**. The Radxa will begin overriding your stick inputs and automatically steer the drone to keep the locked target in the center of the screen.

## 2. FPV Goggles (MSP OSD)
Because you enabled MSP DisplayPort (Canvas Mode) on the UART, you don't need a computer screen to see what the AI is doing. The AI draws directly onto your FPV Goggles.

*   When you are flying normally, you will see a small **crosshair `[ + ]`** in the center of your screen. This is the "Aiming Reticle".
*   To lock onto a target, manually fly the drone so the target is inside the crosshair, then flip **CH7 HIGH**.
*   You will see a box appear around the target with the text `TGT#99`. 
*   If the box stays on the target as it moves, you have a solid lock!

## 3. Engaging the Autopilot
Once you have a solid lock on a target (you see the box tracking it in your goggles):
1.  Flip the **CH6 Switch HIGH**.
2.  The AI will instantly take over the Pitch, Roll, and Yaw controls.
3.  *Note:* You must still manage the throttle manually unless you have set up a barometer altitude hold!
4.  **SAFETY OVERRIDE:** If the drone does something unexpected, instantly flip **CH6 LOW**. The Radxa will immediately drop the override, and you will regain 100% manual control of the drone.

---

# Automatic Startup (Headless Battery Mode)

To make the Radxa automatically start the AI software the moment you plug in your LiPo drone battery, we need to create a `systemd` background service.

### Step 1: Run the Install Script
I have created a new script in your project folder called `install_service.sh`. 
Connect your Radxa to your PC, push the latest code, and run it:

```bash
# On your PC:
scp -r ./interceptorAIModule radxa@192.168.0.105:~/

# SSH into the Radxa:
ssh radxa@192.168.0.105
cd ~/interceptorAIModule
chmod +x scripts/install_service.sh
sudo ./scripts/install_service.sh
```

### Step 2: How it Works
*   The moment the Radxa gets power from the drone battery, Linux will boot.
*   Once Linux finishes booting (~45 seconds), the `triveni-ai.service` will automatically launch `main.py` in the background.
*   The camera will turn on, the AI will load, and you will see the crosshair appear in your FPV goggles, meaning it is ready to fly.

### Step 3: Checking the Status in the Field
If you ever SSH into the Radxa in the field (e.g. from your phone's hotspot) and want to see the AI's live terminal output while it's running in the background, type:
```bash
journalctl -u triveni-ai.service -f
```
*(Press `Ctrl+C` to exit the logs).*
