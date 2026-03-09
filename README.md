# cerbox-gx-cpp

A C++17 library to read info from a Victron CerboGX unit over MQTT

## Dependencies

### Linux (apt)

```sh
sudo apt install mosquitto mosquitto-clients
```

### MacOSX
```sh
brew install mosquitto
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

