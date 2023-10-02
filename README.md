# STM32H745 Starting Repo
Flash target

```
cargo flash --chip STM32H745ZITx
```

## Debugging in VSCode

Follow this guide: https://s2e-systems.github.io/Debugging-embedded-Rust-programs-using-VS-Code/

Installing GDB:

```bash
sudo apt install -y gdb-multiarch openocd
sudo ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb
```

Config file source: https://github.com/openocd-org/openocd/blob/master/tcl/board/st_nucleo_h745zi.cfg
