# Protocol

# Sequences

## Start

|   Actual  |           Next          |
| SDA | SCL | SDA | SCL |    State    |
|-----|-----|-----|-----|-------------|
|  0  |  0  |  1  |     |             |
|  0  |  1  |     |  0  |             |
|  1  |  0  |     |  1  |             |
|  1  |  1  |  0  |     | START2/IDLE |


## Stop
should not require any prerequisite

|   Actual  |           Next          |
| SDA | SCL | SDA | SCL |    State    |
|-----|-----|-----|-----|-------------|
|  0  |  0  |     |  1  |             |
|  0  |  1  |  1  |     |             |
|  1  |  0  |  0  |     |             |
|  1  |  1  |     |  0  |    IDLE     |

## Bit b
Prerequisite: SCL==0

|   |   Actual  |           Next          |
| b | SDA | SCL | SDA | SCL |    State    |
|---|-----|-----|-----|-----|-------------|
| 1 |  0  |  0  |  1  |     |             |
| x |  x  |  1  |     |  0  |             |
| 0 |  1  |  0  |  0  |     |             |
| 0 |  1  |  0  |  0  |     |             |
| 0 |  0  |  0  |     |  1  |    IDLE     |
| 1 |  1  |  0  |     |  1  |    IDLE     |

## Bit b with clock stretch 
Prerequisite: SCL==1

|   |       Actual        |           Next          |
| b | SDA | SCL_i | SCL_0 | SDA | SCL |    State    |
|---|-----|-------|-------|-----|-----|-------------|
| 1 |  0  |   0   |   0   |  1  |     |             |
| x |  x  |   1   |   1   |     |  0  |             |
| 0 |  1  |   0   |   0   |  0  |     |             |
| x |  b  |   0   |   0   |     |  1  |    IDLE     |
| 1 |  0  |   1   |   0   |     |     |  HW_FAULT   |
| x |  x  |   0   |   1   |     |     |             |
| 0 |  1  |   1   |   0   |     |     |  HW_FAULT   |
| x |  b  |   1   |   0   |     |     |  HW_FAULT   |


## Recovery / Alternate STOP
Used when the I2C bus is blocked. This can happen if extra or missing bits (noise, buggy device).
In this case, the SDA line stays at 0, preventing the generation of a STOP then a START.
A recovery option is to toggle SCL line until SDA is 1, then perform the STOP condition

|     Actual          |           Next          |
| SDA_i | SDA_o | SCL | SDA | SCL |    State    |
|-------|-------|-----|-----|-----|-------------|
|   0   |   0   |  1  |  1  |     |             |
|   0   |   0   |  0  |     |  1  |             |
|   0   |   1   |  1  |     |  0  |             |
|   0   |   1   |  0  |  0  |     |             |
|   1   |   0   |  1  |     |     |  HW_FAULT   |
|   1   |   0   |  0  |     |     |  HW_FAULT   |
|   1   |   1   |  1  |     |     |    IDLE     |
|   1   |   1   |  0  |  0  |     |             |
