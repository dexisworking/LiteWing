# LiteWing ESP-IDF Environment Activation Script
# Configured for ESP-IDF v5.5.6 and D:\.espressif toolchains

$Env:IDF_TOOLS_PATH = "D:\.espressif"
$Env:IDF_PATH = "C:\Espressif\frameworks\esp-idf-v5.5.6"
$Env:ESP_IDF_VERSION = "5.5"
$Env:IDF_PYTHON_ENV_PATH = "D:\.espressif\python_env\idf5.5_py3.11_env"
$Env:OPENOCD_SCRIPTS = "D:\.espressif\tools\openocd-esp32\v0.12.0-esp32-20260831\openocd-esp32\share\openocd\scripts"
$Env:IDF_CCACHE_ENABLE = "1"
$Env:ESP_ROM_ELF_DIR = "D:\.espressif\tools\esp-rom-elfs\20260528\"

$Env:PATH = "D:\.espressif\python_env\idf5.5_py3.11_env\Scripts;C:\Espressif\frameworks\esp-idf-v5.5.6\components\espcoredump;C:\Espressif\frameworks\esp-idf-v5.5.6\components\partition_table;C:\Espressif\frameworks\esp-idf-v5.5.6\components\app_update;D:\.espressif\tools\xtensa-esp-elf-gdb\17.1_20260402\xtensa-esp-elf-gdb\bin;D:\.espressif\tools\xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin;D:\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20260121\riscv32-esp-elf\bin;D:\.espressif\tools\esp32ulp-elf\2.38_20240113\esp32ulp-elf\bin;D:\.espressif\tools\cmake\3.30.2\bin;D:\.espressif\tools\openocd-esp32\v0.12.0-esp32-20260831\openocd-esp32\bin;D:\.espressif\tools\ninja\1.12.1\;D:\.espressif\tools\idf-exe\1.0.3\;D:\.espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64;D:\.espressif\tools\dfu-util\0.11\dfu-util-0.11-win64;C:\Espressif\frameworks\esp-idf-v5.5.6\tools;C:\Espressif\tools\idf-git\2.44.0\cmd;$Env:PATH"

function global:idf.py { & "D:\.espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" "C:\Espressif\frameworks\esp-idf-v5.5.6\tools\idf.py" @args }
function global:esptool.py { & "D:\.espressif\python_env\idf5.5_py3.11_env\Scripts\python.exe" -m esptool @args }

Write-Host "LiteWing ESP-IDF v5.5.6 environment active!" -ForegroundColor Green
Write-Host "Tools path: $Env:IDF_TOOLS_PATH" -ForegroundColor Cyan
