# Software Configuration via config_builder

T.R.I.V.E.N.I is configured entirely via the `config.json` file. To make editing safe and intuitive, a visual configuration builder is provided.

## Launching the Config Builder
You can run the configuration builder on any PC (or on the Radxa if you forward the port):

```bash
python3 config_builder.py
```
This starts a local web server at `http://127.0.0.1:8080`. Open this URL in any web browser to see the dark-themed "JS aerial" dashboard.

## Channel Mappings
The system needs to know which channels on your radio correspond to which axes. 
By default, the system assumes an **AETR** layout:
- **Roll Channel Index:** `0` (Channel 1)
- **Pitch Channel Index:** `1` (Channel 2)
- **Throttle Channel Index:** `2` (Channel 3)
- **Yaw Channel Index:** `3` (Channel 4)

If your radio uses **TAER**, change these indices accordingly (Throttle = 0, Roll = 1, Pitch = 2, Yaw = 3).

### Auxiliary Switches
You must map two separate switches on your radio to trigger the AI:
- **Lock Switch Channel:** Defaults to `6` (AUX 3). Flipping this high tells the AI to lock onto the nearest target in the crosshairs.
- **Follow Switch Channel:** Defaults to `5` (AUX 2). Flipping this high allows the AI to send `MSP_SET_RAW_RC` packets to physically move the drone.

## Stick Calibration & Inversions
The Radxa needs to know the endpoints of your radio to generate valid control packets.
- **RC Mid:** Usually 1500us.
- **RC Min / Max:** Usually 1000us / 2000us.
- **Expo:** Smooths out the AI's PID reactions. Default is `0.85`.
- **Inversions:** If the drone tracks the target but flies *away* from it, change the `pitch_dir` or `yaw_dir` from `1.0` to `-1.0`.
