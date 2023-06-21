# Regmap

Similarly to Linux's counterpart, regmap simplifies the access to deivice registers over I2C busses.

Currently, this package only allows to write device registers, and simulate reads but the read result is not used. It is useful when simple I2C devices need initialization but adding a CPU and Software to do so is not desired.
In the future, regmap will enable to use the register read value and ACK status.

## Usage
See examples
