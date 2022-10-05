# Phoenix Throttle ECU

This is the codebase for Phoenix's throttle ECU, part of the drive by wire subsystem.

See [the design doc](https://github.com/ISC-Project-Phoenix/design/blob/main/software/Throttle.md) for more info.

## Pinout

- Can rx: pb8
- Can tx: pb9
- Dac out: pa4

Transceiver is powered with 3v3 and Gnd.

The blue LED will blink with each Can message received.

## Building

This codebase is designed to be run on an ST Nucleo-f767zi.

1. Install the [Rust toolchain](https://www.rust-lang.org/learn/get-started).
2. `cargo install probe-run` 
3. `cargo install flip-link`
4. `rustup target install thumbv7em-none-eabihf`
5. build the repo with `cargo build` or `cargo build --release` and flash manually, or debug with
`cargo run`.
