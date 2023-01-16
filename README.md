# C++23 embedded experiments

This project was started as an experiment. The main goal is to create
hi-level, memory-safe zero-cost abstractions using modern C++23. This
repository is not intended to transform into a library but rather be a
collection of experiments that will eventually become a solid baggage of
experience (which will hopefully lead me to a point where I can start designing
a modern C++ framework for microcontrollers). 

This project is at it's very early stage. Things may be brutally rewritten,
reorganized without any remorse.

## Basic assumptions and goals

The final library/framework should comply with the following assumptions:

###  Security first

It must be hard (or impossible) to use the framework wrong. This can be
achieved by using strong types and concepts extensively.

In the `C` programming world - It is very common to fall into a trap and mix
bits or registers. For example, given the following `C` code for the AVR
microcontroller:

```C
void adc_start_conversion() {
    ADCSRB |= (1 << ADSC); 
}
```

The above function was supposed to start the analog-to-digital conversion, but
instead will force the ADC multiplexer to select the negative input to the
analog comparator. All because of the fact that the programmer mixed the
`ADCSRA` and `ADCSRB` registers.

We can easily avoid this kind of errors, C++23 comes with a handy tools which
may help: `std::is_scoped_enum` and `std::to_underlying`.

### True zero-cost abstractions

The library must use a true, instruction-level test system to ensure maximum
performance.

This can be achieved by compiling dedicated tests with `-ffunction-sections`
and `-fdata-sections` and comparing each section of the resulting binaries with
a predefined set of expected instructions.

### Universal access to registers

The library must provide a universal way to work with *registers*. Both for
MCU's registers and registers of external devices.

I already provided a working proof-of-concept which looks like this:

```C++
using gpio_oe = rw_reg<addrs::sio_base, 0x20>;
using gpio_oe_set = rw_reg<addrs::sio_base, 0x024>;
using gpio_oe_clr = rw_reg<addrs::sio_base, 0x028>;
using gpio_oe_xor = rw_reg<addrs::sio_base, 0x02c>;
```

The user can access these registers with static functions:

```C++
registers::gpio_oe_set::set_value(1 << 5);
```

Or create a higher level abstractions around it:

```C++
gpio::pin<pins::gpio25> led0;
led0.set_as_output();
led0.toggle();
```

The following C++23 features may be helpful:

#### Multidimensional subscript operator ([P2128R6](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2128r6.pdf))

The
[P2128R6](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2128r6.pdf)
and
[2589R0](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2589r0.pdf)
papers may allow us to define a multidimensional subscript operator
(`operator[](auto... bits)`). As a result, we may use the following syntax to
access registers:

```C++
registers::gpio_oe[bit::1, bit::2, bit::5] = 1;
registers::gpio_oe[bit::1, bit::2, bit::5] = state::high;

// or simpler:
registers::gpio_oe[1, 2, 5] = 1;
```

#### Reusable device drivers

Device drivers must by provided in a form which will be reusable across many
different CPU architectures and many different communication methods, thus they
need to be written as "*what to do*" rather than "*how to do it*".

For example, the common `HD44780` LCD controller may be connected to
software-controlled GPIOs or via an I2C converter or even to a dedicated MCU's
peripheral designed to interface with parallel memory modules.

The "*ideal*" situation will be to provide a driver with the following
interface:

```C++
constexpr auto interface_descriptor = interface::gpio::for<drivers::lcd::hd44780>{
    .mode = drivers::lcd::hd44780::4bit,
    .register_select = platform::pins::gpio10,
    .read_write = platform::pins::gpio11,
    .enable = platform::pins::gpio12,
    .data0 = platform::pins::gpio13,
    .data1 = platform::pins::gpio13,
    .data2 = platform::pins::gpio13,
    .data3 = platform::pins::gpio13,
};

drivers::lcd::hd44780 main_lcd{interface::gpio, interface_descriptor};
drivers::lcd::hd44780 second_lcd{interface::i2c,
                                 interface::i2c::for<drivers::lcd::hd44780>{
                                    // i2c parameters
                                 }};

std::print(main_lcd, "Hello world...");
std::print(second_lcd, "Hello world again...");

```

#### Convenient input/output

The library must provide a seamless integration with the great `std::print`
(see
[P2093R14](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2093r14.html)),
for example:

```C++
std::print(uart0, "Hello world from the microcontroller! My CPUID is: {}", platform::cpuid)
```

### Code-completion friendly

The library must be developer-friendly, thus all types, globals, function names
must use suitable prefixes to allow developer to use the library without
digging into the documentation. For example use `m_member` prefix instead of
`member_` postfix while working with member variables.

## Build system

The library must use modern, bloat-free build system. My favourite one is
`Meson`.

### Building

The project must be built using the Mesons's cross build environment.

To build the project, use the following command:

```console
$ meson setup --cross-file cross/armv6-m/rp2040.txt build/
$ ninja -C build/ -v
```

The following file should be created: `mcupp-example.elf` inside the
`build/src/` directory.

