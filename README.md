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
