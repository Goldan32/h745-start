# STM32H745 Starting Repo
Flash target

```
cargo flash --chip STM32H745ZITx
```

or for single core

```
cargo flash --elf target/thumbv7em-none-eabihf/release/blink-1 --chip STM32H745ZITx
```

Microamp magic:

```
cargo +nightly microamp --bin blink --release -v
```

There is an error where project does not work after flashing, to fix it:

- From the `nucleo-h7xx` project use `cargo build --example blinky` and `cargo flash --chip STM32H745ZITx --example blinky`
- Disconnect and reconnect board from power source
- Flash `merge.hex` after using `make` to build this project

## Debugging in VSCode

Follow this guide: https://s2e-systems.github.io/Debugging-embedded-Rust-programs-using-VS-Code/

Installing GDB:

```bash
sudo apt install -y gdb-multiarch openocd
sudo ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb
```

Config file source: https://github.com/openocd-org/openocd/blob/master/tcl/board/st_nucleo_h745zi.cfg
