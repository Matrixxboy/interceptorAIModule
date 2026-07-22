# Structure Project for PyInstaller and Data Persistence

The project currently uses `__file__` to resolve the locations of configuration files, presets, logs, databases, and models. When bundled with PyInstaller, `__file__` resolves to a temporary folder (`sys._MEIPASS`) that gets deleted when the program exits. This means any presets or logs saved during runtime will be lost. Furthermore, standard software on Windows stores user-specific data in the AppData directory.

## Proposed Changes

### 1. Introduce a Central Paths Module
Create a new file `interceptorAIModule/paths.py` that will intelligently determine where the application is running from and where data should be stored.

#### [NEW] [paths.py](file:///d:/interceptor/interceptorAIModule/paths.py)
This module will define:
- `BUNDLE_DIR`: Resolves to `sys._MEIPASS` if running as a PyInstaller EXE, or the project root if running from source. This is for read-only bundled assets (e.g., `models/`).
- `APP_DATA_DIR`: Resolves to `%APPDATA%\ArjunaGCS` on Windows (or `~/.config/ArjunaGCS` on Linux/Mac). This is the root for all persistent user data.
- Read-only Paths: `MODELS_DIR`.
- Writable Paths: `PRESETS_DIR`, `LOGS_DIR`, `DATA_DIR` (for targets database), `PLUGINS_DIR`.

### 2. Refactor Existing Code to Use `paths.py`
We will replace hardcoded `Path(__file__)` logic across the codebase with imports from `paths.py`.

#### [MODIFY] [config.py](file:///d:/interceptor/interceptorAIModule/config.py)
- Replace `ROOT`, `MODELS_DIR`, `PRESETS_DIR` with imports from `paths.py`.

#### [MODIFY] [system_logger.py](file:///d:/interceptor/interceptorAIModule/sys_logging/system_logger.py)
- Replace local `LOGS_DIR` derivation with `from paths import LOGS_DIR`.

#### [MODIFY] [telemetry_logger.py](file:///d:/interceptor/interceptorAIModule/telemetry/telemetry_logger.py)
- Replace local `LOGS_DIR` derivation with `from paths import LOGS_DIR`.

#### [MODIFY] [target_store.py](file:///d:/interceptor/interceptorAIModule/database/target_store.py)
- Replace local `DATA_DIR` derivation with `from paths import DATA_DIR`.

#### [MODIFY] [plugin_manager.py](file:///d:/interceptor/interceptorAIModule/core/plugin_manager.py)
- Replace local `project_root` derivation with `from paths import PLUGINS_DIR`.

#### [MODIFY] [config_manager.py](file:///d:/interceptor/interceptorAIModule/core/config_manager.py)
- Ensure configuration JSONs are saved to `APP_DATA_DIR/configs`.

### 3. Create PyInstaller Build Assets
We need to provide a standard way to build the executable.

#### [NEW] [build_exe.py](file:///d:/interceptor/build_exe.py)
A Python script to cleanly invoke PyInstaller with all the necessary arguments:
- `--name Arjuna`
- `--add-data "interceptorAIModule/models:models"` (Bundles the AI models)
- `--add-data "interceptorAIModule/calibration.json:."` (Bundles calibration info)
- Hides the console window (if desired, though for this we can keep a toggle or just use a windowed mode flag).

## User Review Required

> [!IMPORTANT]
> - By default, PyInstaller will create a single executable file. When it runs, it extracts itself to a temporary folder (`sys._MEIPASS`).
> - The new data storage location for your presets, logs, and database will be in your Windows AppData folder (e.g. `C:\Users\<username>\AppData\Roaming\ArjunaGCS`). This is the correct, standard behavior for Windows applications. 
> 
> Are you okay with the data moving to the `AppData` folder?

## Verification Plan

### Automated Tests
- N/A

### Manual Verification
- We will generate the PyInstaller build and ensure the EXE successfully starts up.
- We will verify that presets and logs are successfully written to the `AppData/Roaming/ArjunaGCS` folder.
