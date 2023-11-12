# STM32H745 Dual-Core Project

## Build Project

### Native environment

Building the project requires the following setup on the host machine, which preferebly runs a Debian-like linux distribution:

**1. Install the following apt packages**
```
libudev-dev
gdb-multiarch
picocom
openocd
curl
stlink-tools
binutils-multiarch
xxd
binutils-arm-none-eabi
srecord
```

**2. Install Rust and Cargo**

**3. Install additional Rust tools**

```bash
rustup component add llvm-tools-preview
rustup target add thumbv7em-none-eabihf
rustup install nightly
rustup +nightly target add thumbv7em-none-eabihf
cargo install cargo-binutils --vers 0.3.6
cargo install cargo-flash
cargo install cargo-make
cargo install microamp-tools --git https://github.com/rtfm-rs/microamp
```

**4. Use the makefile from the project to build images**

```bash
cargo make
```

After this step there are multiple outputs in the `artifacts` directory.

- image-0.bin and image-1.bin: Binary image for each core
- image-0.elf and image-1.elf: ELF image for each core
- image-0.hex and image-1.hex: HEX image for each core
- merge.hex: A merged HEX image that contains code for both cores

### Docker

**1. Install the following dependencies**

- VSCode, with extensions
  - [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- Docker

**2. Open the project with VSCode**

**3. Reopen project in container**

Press *Ctrl+Shift+P* and start typing `Dev Container: Reopen in Container` and select that option.

**4. Build using the command line**

The container is built, use the shell in VSCode to build the project using the makefile.

```bash
cargo make
```

## Flash image to target

### Single core - single core project (earlier commits)

Use `cargo flash` utility

```bash
cargo flash --chip STM32H745ZITx
```

### Single core - dual core project

Use `cargo flash` utility

```bash
# Core 0
cargo flash --elf target/thumbv7em-none-eabihf/release/h745-dual-core-0 --chip STM32H745ZITx

# Core 1
cargo flash --elf target/thumbv7em-none-eabihf/release/h745-dual-core-1 --chip STM32H745ZITx
```

### Dual core (merged HEX image)

Use the STM32CubeProgrammer utility provided by STM. Tool can be downloaded from their [website](https://www.st.com/en/development-tools/stm32cubeprog.html#get-software).



### Known errors with flashing

~~There is an error where project does not work after flashing, to fix it:~~This issue was fixed in this commit.

### Known errors with the program

Soft reset or even power reset does not reset shared variables.

## Debugging in VSCode

Follow this guide: https://s2e-systems.github.io/Debugging-embedded-Rust-programs-using-VS-Code/

Installing GDB:

```bash
sudo apt install -y gdb-multiarch openocd
sudo ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb
```

Config file source: https://github.com/openocd-org/openocd/blob/master/tcl/board/st_nucleo_h745zi.cfg
