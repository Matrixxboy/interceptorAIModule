# Radxa OS & Software Installation

This guide covers preparing the Radxa Zero 3W operating system and installing the lightweight T.R.I.V.E.N.I software stack (Optimized for 2GB RAM).

## 1. Flash the OS
1. Download the **5.10 Kernel: radxa-zero3_debian_bullseye_xfce_b6** image. 
   *(Note: The 5.10 kernel is highly recommended over 6.1 because Rockchip's MIPI camera drivers and hardware acceleration are much more stable on 5.10).*
2. Flash the OS to a high-speed MicroSD card using BalenaEtcher or Rufus.
3. Boot the Radxa, connect to WiFi/Ethernet, and SSH into the board.
4. **CRITICAL:** Disable the XFCE desktop environment to save RAM! Run this command:
   ```bash
   sudo systemctl set-default multi-user.target
   sudo reboot
   ```
   *This ensures the Radxa boots in headless/CLI mode, keeping maximum RAM available for the AI.*

## 2. Install System Dependencies
Run the following commands to update the system and install required video/camera drivers:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-opencv v4l-utils gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

## 3. Verify the Camera
Ensure the Raspberry Pi Camera is detected by the OS:
```bash
v4l2-ctl --list-devices
```
You should see `/dev/video0` associated with the MIPI or Rockchip ISP.

## 4. Install T.R.I.V.E.N.I
Clone or copy the T.R.I.V.E.N.I codebase to the Radxa.

```bash
cd ~
# Copy the files over, then install the minimal requirements
pip3 install -r requirements.txt
```

> [!IMPORTANT]  
> Do **NOT** install `torch` or `ultralytics`. The 2GB Radxa will crash due to Out-Of-Memory errors. We use OpenCV DNN for inference.

## 5. Exporting YOLO Weights to ONNX (ON YOUR PC)
Since the Radxa cannot run PyTorch, you must convert your trained `.pt` weights to `.onnx` on your **Desktop PC** before copying them to the Radxa.

On your desktop PC:
```bash
pip install ultralytics
yolo export model=drone_missile_best.pt format=onnx imgsz=640
```
This generates `drone_missile_best.onnx`. Copy this `.onnx` file into the `models/` directory on the Radxa.

## 6. Run the Daemon
To start the autonomous tracking system:
```bash
python3 main.py
```
To run it automatically on boot, add it to a `systemd` service.
