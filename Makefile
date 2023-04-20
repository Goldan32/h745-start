ARTIFACT_DIR := artifacts
HEX_FILE := $(ARTIFACT_DIR)/image.hex
ELF_FILE := $(ARTIFACT_DIR)/image.elf
BIN_FILE := $(ARTIFACT_DIR)/image.bin
ORIGINAL_ELF := target/thumbv7em-none-eabihf/debug/blink

.PHONY: all clean crate

default: all

all: $(BIN_FILE)

$(ELF_FILE): crate | $(ARTIFACT_DIR)
	cp "$(ORIGINAL_ELF)" "$@"

$(HEX_FILE): $(ELF_FILE)
	objcopy -O ihex "$<" "$@"

$(BIN_FILE): $(HEX_FILE)
	xxd -r -p "$<" "$@"

crate:
	cargo build

$(ARTIFACT_DIR):
	mkdir -p "$@"

clean:
	rm -rf $(ARTIFACT_DIR)
	cargo clean
