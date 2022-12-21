# Phoenix AEB ECU

This repo is an implementation of the AEB ECU for Project Phoenix.  

See [the design doc](https://github.com/ISC-Project-Phoenix/design/blob/main/software/embed/AEB.md) for more info.
## Pinout 

- Lidar UART rx: pd0

- Lidar PWM out: pe9

- CAN rx: pb8

- CAN tx: pb9

CAN transceiver is powered with 3v3 and Gnd.

**LD06 connector order, left to right:**

UART tx, PWM in, GND, 5V

## Building
This codebase is designed to be run on an ST Nucleo-f767zi, along with an LD06 lidar.

1. Install the [Rust toolchain](https://www.rust-lang.org/learn/get-started).
2. `cargo install probe-run`
3. `cargo install flip-link`
4. `rustup target install thumbv7em-none-eabihf`
5. build the repo with `cargo build` or `cargo build --release` and flash manually, or debug with
   `cargo run`.