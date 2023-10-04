FROM rust:1.72

# Programs installed with apt
RUN apt-get update && apt-get install -y \
    libudev-dev \
    gdb-multiarch \
    picocom \
    openocd \
    curl \
    make \
    stlink-tools \
    binutils-multiarch \
    xxd \
    binutils-arm-none-eabi \
    srecord

# Cleaning up to reduce image size
RUN apt-get autoremove -y
RUN apt-get clean -y
RUN apt-get autoclean -y

# Creating a soft link so a vscode gdb debugger doesn't get confused with naming
RUN ln -s /usr/bin/gdb-multiarch /usr/bin/arm-none-eabi-gdb

# Setting the user to non-root privlages
RUN useradd --create-home --shell /bin/bash rustacean
USER rustacean

# Install embedded rust tools
RUN rustup component add llvm-tools-preview
RUN rustup target add thumbv7em-none-eabihf
RUN rustup install nightly
RUN rustup +nightly target add thumbv7em-none-eabihf
RUN cargo install cargo-binutils --vers 0.3.6
RUN cargo install cargo-flash
RUN cargo install microamp-tools --git https://github.com/rtfm-rs/microamp
RUN cargo install cargo-make

WORKDIR /work
