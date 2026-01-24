# MAARCO

This is the code for the MAARCO project. It involves reading GPS RTK data from a serial port, displaying it in a terminal UI, and optionally connecting to an NTRIP server for real-time corrections.

We are using a Raspberry Pi Zero 2W as the main hardware platform.

All data is also logged to a file. The file is a CSV, which you can find under `logs/`.

## Project structure

- `src/`: Contains the Rust source code for the project.
- `examples/`: Contains example Python scripts for decoding log files and streaming NMEA data. Also includes some Rust code.
- `logs/`: Directory where CSV log files are stored.
- `field_tests/`: This is where the data from field tests is stored. The data can be decoded using the scripts in `examples/decode_log_file.py`.


## How to run

Since this project uses both Rust and Python, ensure you have both installed. For python, you need to install [`uv`](https://docs.astral.sh/uv/getting-started/installation/) to manage dependencies.

To install Rust, follow the instructions at [rustup.rs](https://rustup.rs/).

1. Clone the repository:
   ```bash
   git clone https://github.com/harshil21/MAARCO.git && cd MAARCO
   ```

2. Install Python dependencies:
   ```bash
   uv sync
   ```

3. Build the Rust project:
   ```bash
    cargo build --release
    ``` 

Now, if you want to run the main application on the Raspberry Pi, you would need to compile the code for the ARM architecture. You can do this by setting up a cross-compilation environment. It is highly recommended to use your computer for cross compiling, as compiling directly on the Raspberry Pi can be *very* slow.

You can cross compile via cargo:

1. First, install the ARM target:
    ```bash
    rustup target add aarch64-unknown-linux-gnu
    ```
   
2. Next, install the necessary linker. On Ubuntu or [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) (you should get the Ubuntu distribution), you can do this via:
    ```bash
      sudo apt-get install build-essential gcc-aarch64-linux-gnu
      ```
3. Now, you can build the project for the Pi Zero 2W:
    ```bash
    cargo build --target aarch64-unknown-linux-gnu --release
      ```

4. You should now have a binary located at `target/aarch64-unknown-linux-gnu/release/maarco`. You should copy this binary to the Raspberry Pi, e.g. via
`scp`:
```bash
scp target/aarch64-unknown-linux-gnu/release/maarco pi4b@<RASPBERRY_PI_IP_ADDRESS>:~/
```

5. Finally, run the application on the Raspberry Pi:
    ```bash
    ./maarco --ntrip-mount MOUNTPOINT --log-file logs/test_1.csv
    ```

Replace `MOUNTPOINT` with your actual NTRIP mount point
(the list of all NTRIP mount points can be found [here](http://rtk2go.com:2101/SNIP::STATUS)).

You should typically use the `VMAX-LAND-1` mount point. This is located in [Baybrook, NC](http://rtk2go.com:2101/SNIP::BASEandUSERMAP?baseName=VMAX-LAND-1&tk=nFghcsAw9xJ3rpf6qwPE) and is good for testing if you are within 10 miles of that location.

For locations further away, you should choose a mount point closer to your location, otherwise the RTK corrections will not be effective. You can browse the list of mount points in the link above to find one near you. If there are none, you should
set up your own RTK base station (see [Base station setup](#base-station-setup) below).

If you don't have one, you can omit the `--ntrip-mount` argument, and the application will run
without NTRIP support (so you will not get RTK corrections).


### Running without the Pi:

You can also run the application on your computer if you have a GPS device connected via USB or serial port. Just make sure to specify the correct serial port in the code (currently set to `/dev/ttyUSB0`).


### Running the example scripts:

You can run the example Python scripts to decode log files or stream NMEA data. For example, to decode a log file:

```bash
uv run --script examples/decode_log_file.py
```

Or to run a Rust example:

```bash
cargo run --example file_name[no .rs]
```

## Base station setup

TODO!
