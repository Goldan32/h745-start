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

