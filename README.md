# cerbox-gx-cpp

A C++17 library to read info from a Victron CerboGX unit over Modbus-TCP

## Dependencies

### Linux (apt)

```sh
sudo apt install mosquitto mosquitto-clients libmodbus-dev
```

### MacOSX
```sh
brew install mosquitto libmodbus
```

## Build

```sh
mkdir build
cd build
cmake ..
make
```

## Demo

```sh
./cerbo_gx_test --ip <cerbo_ip>
```

