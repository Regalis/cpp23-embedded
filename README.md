# C++23 embedded experiments

## TODO: update README, write few words about the project

## Build system

### Building

The project must be built using the Mesons's cross build environment.

To build the project, use the following command:

```console
$ meson setup --cross-file cross/armv6-m/rp2040.txt build/
$ ninja -C build/ -v
```

The following file should be created: `mcupp-example.elf` inside the
`build/src/` directory.

