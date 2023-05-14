ARTIFACT_DIR := artifacts
HEX_FILE_0 := $(ARTIFACT_DIR)/image-0.hex
ELF_FILE_0 := $(ARTIFACT_DIR)/image-0.elf
BIN_FILE_0 := $(ARTIFACT_DIR)/image-0.bin
HEX_FILE_1 := $(ARTIFACT_DIR)/image-1.hex
ELF_FILE_1 := $(ARTIFACT_DIR)/image-1.elf
BIN_FILE_1 := $(ARTIFACT_DIR)/image-1.bin
ORIGINAL_ELF_0 := target/thumbv7em-none-eabihf/release/blink-0
ORIGINAL_ELF_1 := target/thumbv7em-none-eabihf/release/blink-1
MERGE_HEX := $(ARTIFACT_DIR)/merge.hex

.PHONY: all clean crate

default: all

all: $(BIN_FILE_0) $(BIN_FILE_1) $(MERGE_HEX)

$(ELF_FILE_0): crate | $(ARTIFACT_DIR)
	cp "$(ORIGINAL_ELF_0)" "$@"

$(ELF_FILE_1): crate | $(ARTIFACT_DIR)
	cp "$(ORIGINAL_ELF_1)" "$@"

$(HEX_FILE_0): $(ELF_FILE_0)
	objcopy -O ihex "$<" "$@"

$(HEX_FILE_1): $(ELF_FILE_1)
	objcopy -O ihex "$<" "$@"

$(BIN_FILE_0): $(HEX_FILE_0)
	xxd -r -p "$<" "$@"

$(BIN_FILE_1): $(HEX_FILE_1)
	xxd -r -p "$<" "$@"

$(MERGE_HEX): $(HEX_FILE_0) $(HEX_FILE_1)
	srec_cat $(HEX_FILE_0) -intel $(HEX_FILE_1) -intel -o $@ -intel -Line_Length 44

crate:
	cargo +nightly microamp --bin blink --release -v


$(ARTIFACT_DIR):
	mkdir -p "$@"

clean:
	rm -rf $(ARTIFACT_DIR)
	cargo clean
