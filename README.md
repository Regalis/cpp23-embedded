# C++23 embedded experiments

This project was started as an experiment. The main goal is to create
hi-level, memory-safe zero-cost abstractions using modern C++23. This
repository is not intended to transform into a library but rather be a
collection of experiments that will eventually become a solid baggage of
experience (which will hopefully lead me to a point where I can start designing
a modern C++ framework for microcontrollers). 

This project is at it's very early stage. Things may be brutally rewritten,
reorganized without any remorse.

# MCU selected for this experiment

I have decided to use `RP2040` from the **Raspberry Pi** as a main
microcontroller for this experiment. It was a rather difficult (but necessary)
choice because the `RP2040` is not easy to use when it comes to real bare metal
programming. Mainly because it has a complicated boot method due to the lack of
built-in flash memory. This significantly raised the entry barrier.

Everything you see in this repository is a result of **real bare metal
programming** in C++23. Everything is written from scratch without **any
external dependencies**. I mean everything... Including **linker scripts**,
**memory map** and even **bootloader**.

Things that are up and running as for today:

* booting from the external flash memory using hand-written bootloader (stage #2),
* running the system at 125MHz:
    * configuring a crystal oscillator (XOSC),
    * configuring PLLs,
    * configuring all system clocks (`clk_gpout{0..3}`, `clk_ref`, `clk_sys`, `clk_peri`, `clk_usb`, `clk_adc`, `clk_rtc`),
* configuring a watchdog timer,
* configuring timers,
* sending/receiving data with UART,
* configuring GPIOs,
* configuring PWMs.

Take a look at [examples/](https://gitlab.com/Regalis/cpp23-embedded/-/tree/master/examples) for a list of **working examples**.

Teaser:

```c++
// (cut)
int main()
{
    gpio::pin<platform::pins::gpio25> led0;
    led0.function_select(gpio::functions::sio);
    led0.set_as_output();

    // Prepere descriptor for the selected interface
    constexpr auto descriptor =
      hd44780::interfaces::gpio4_bit{.register_select = platform::pins::gpio16,
                                     .enable = platform::pins::gpio17,
                                     .data4 = platform::pins::gpio18,
                                     .data5 = platform::pins::gpio19,
                                     .data6 = platform::pins::gpio20,
                                     .data7 = platform::pins::gpio21};

    // Define your LCD hardware layout
    constexpr auto configuration = hd44780::configuration{
      .columns = 16, .lines = 2, .font_size = hd44780::font::font_5x8};

    // Get type of the driver based on your descriptor and configuration
    using lcd =
      hd44780::hd44780<hd44780::interface_for<descriptor>, configuration>;

    timer::delay(500ms);

    // Initialize both MCU interface (in this case - the GPIOs) and the LCD
    // itself
    lcd::init();

    lcd::soft_puts("Hello world");
    lcd::cursor_goto(0, 1);
    lcd::soft_puts("blog.regalis.tech");

    while (true) {
        led0.toggle();
        timer::delay(250ms);
    }
// (cut)
```

# Basic assumptions and goals

The final library/framework should comply with the following assumptions:

##  Security first

It must be hard (or impossible) to use the framework wrong. This can be
achieved by using strong types and concepts extensively.

In the `C` programming world - It is very common to fall into a trap and mix
bits or registers. For example, given the following `C` code for the AVR
microcontroller:

```c
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

## Ensure correctness

As it comes to correctness - we already have plenty of great tools built right
into the C++. By using `constexpr`, `consteval` and `static_assert` - we can
provide bit-level correctness for all abstractions.

## No preprocessor

I think we came into the point where we do not need any preprocessor
directives - we can replace them all with `constexpr`, `consteval` and a bit of
metaprogramming.

**Note**: To be fair, the only one preprocessor directive you may see in the
codebase is `#include` - this is due to lack of full support for **modules** in
both [Meson build system](https://github.com/mesonbuild/meson/issues/5024) and
**GCC**.

## Absolute no dependencies

The final library/framework should not use any dependencies at all, not even
headers provided by the CPU vendors. 

This is to **enforce maximum security** and ensure that the library/framework
will be **fully controlled by the end users**.

*This is the absolute opposite of what Rust community is doing all over the
place (using a library/framework will almost always lead to downloading
gazillion of dependencies).*

## True zero-cost abstractions

The library must use a true, instruction-level test system to ensure maximum
performance.

This can be achieved by compiling dedicated tests with `-ffunction-sections`
and `-fdata-sections` and comparing each section of the resulting binaries with
a predefined set of expected instructions.

## Universal access to registers

The library must provide a universal way to work with *registers*. Both for
MCU's registers and registers of external devices.

I already provided a working proof-of-concept which looks like this:

```c++
using gpio_oe = rw_reg<addrs::sio_base, 0x020>;
using gpio_oe_set = rw_reg<addrs::sio_base, 0x024>;
using gpio_oe_clr = rw_reg<addrs::sio_base, 0x028>;
using gpio_oe_xor = rw_reg<addrs::sio_base, 0x02c>;
```

The user can access these registers with static functions:

```c++
registers::gpio_oe_set::set_value(1 << 5);
```

Or create a higher level abstractions around it:

```c++
gpio::pin<pins::gpio25> led0;
led0.set_as_output();
led0.toggle();
```

The following C++23 features may be helpful:

### Multidimensional subscript operator ([P2128R6](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2128r6.pdf))

The
[P2128R6](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2021/p2128r6.pdf)
and
[2589R0](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2589r0.pdf)
papers may allow us to define a multidimensional subscript operator
(`operator[](auto... bits)`). As a result, we may use the following syntax to
access registers:

```c++
registers::gpio_oe[bits::bit1, bits::bit2, bits::bit5] = 1;
registers::gpio_oe[bits::bit1, bits::bit2, bits::bit5] = state::high;

// or simpler:
registers::gpio_oe[1, 2, 5] = 1;
```

## Reusable device drivers

Device drivers must by provided in a form which will be reusable across many
different CPU architectures and many different communication methods, thus they
need to be written as "*what to do*" rather than "*how to do it*".

For example, the common `HD44780` LCD controller may be connected to
software-controlled GPIOs or via an I2C converter or even to a dedicated MCU's
peripheral designed to interface with parallel memory modules.

The "*ideal*" situation will be to provide a driver with the following
interface:

```c++
constexpr auto interface_descriptor = interface::gpio::for<drivers::lcd::hd44780>{
    .mode = drivers::lcd::hd44780::four_bits,
    .register_select = platform::pins::gpio10,
    .read_write = platform::pins::gpio11,
    .enable = platform::pins::gpio12,
    .data0 = platform::pins::gpio13,
    .data1 = platform::pins::gpio14,
    .data2 = platform::pins::gpio15,
    .data3 = platform::pins::gpio16,
};

drivers::lcd::hd44780 main_lcd{interface::gpio, interface_descriptor};
drivers::lcd::hd44780 second_lcd{interface::i2c,
                                 interface::i2c::for<drivers::lcd::hd44780>{
                                    // i2c parameters
                                 }};

std::print(main_lcd, "Hello world...");
std::print(second_lcd, "Hello world again...");

```

## Convenient input/output

The library must provide a seamless integration with the great `std::print`
(see
[P2093R14](https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2022/p2093r14.html)),
for example:

```c++
std::print(uart0, "Hello world from the microcontroller! My CPUID is: {}", platform::cpuid);
```

## Code-completion friendly

The library must be developer-friendly, thus all types, globals, function names
must use suitable prefixes to allow developer to use the library without
digging into the documentation. For example use `m_member` prefix instead of
`member_` postfix while working with member variables.

# Build system

The library must use modern, bloat-free build system. My favourite one is
`Meson`.

## Building

The project must be built using the Mesons's cross build environment.

To build the project, use the following command:

```console
$ meson setup --cross-file cross/armv6-m/rp2040.txt build/
$ meson compile -C build/
```

The above commands will build all the examples by default.

## Flashing

Examples are ready to be flashed to the Raspberry Pi Pico board. In order to
generate firmware images (UF2) for the Pico - you will need to convert the
`elf` files into the `uf2` files. I have implemented the tool specificly for
this job. You can download it [here](https://gitlab.com/Regalis/pico-bin2uf2). 

I have prepared the build system in such a way that UF2 files will be
automatically generated as soon as the `regalis-pico-bin2uf2` program is in
your PATH.

### Things to do after installing `regalis-pico-bin2uf2`

The only thing to is to reconfigure your build directory with `meson setup
--reconfigure build/` and to run `meson compile -C build` one again.

Happy hacking!
