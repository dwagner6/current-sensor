# UART Command Reference

The board interface for remote command control happens over UART, therefore there is a set of text commands and responses that can be used to control the pulse parameters.  

## Command format
Commands take the the form of a string in the format `<command> <parameter>` where `<command>` and `<parameter>` are separated by a single `<Space>`.  Response from the board when command is successful is `OK!`, and when the command is unsuccessful `Error!`

## Command reference
| Command | Parameter | Note | 
| ------- | --------- | ---- | 
| `onoff` | `0` or `1` |  Turns output on (`1`) or off (`0`) | 
| `freq` |  `100` - `10000` | Sets frequency of pulses in Hz | 
| `width` | `500` - `1000000` | Sets pulse width in ns | 
| `curr` | `0` - `20000` | Sets current in mA | 
| `vout` | `800` - `15000` | Sets capacitor voltage in mV (Debug command) |
| `adc` | `0` - `3`| Prints the current reading from the selected ADC channel in mA | 
| `raw_adc` | `0` - `3` | Prints the raw voltage reading from selected ADC channel in mV |
| `get_readings` | n/a | Get all 4 adc readings in mA (used for Labview) | 
| `get_settings` | n/a | Get all current settings (used for Labview) | 
| `set_settings` | `<freq> <width> <curr> <onoff> <vout>` | Set all settings at once (used for Labview). Format is string of 5 integers with seperated by space |  

### Example: Set 0.1% duty cycle at 1 kHz, 10 A pulses 
- Set current to 10 A: `curr 10000`   
- Set 1 kHz frequency: `freq 1000`
- Set 1 us pulse width (0.1%): `width 1000` 
- Turn on output: `onoff 1` 
- (Alternatively): `set_settings 1000 1000 10000 1 0`
- Get measured pulse current from 10 A range: `adc 2` 
- Check raw adc voltage on 10 A range: `raw_adc 2`

## Debug mode
To slow down the UART timeout to be able to send commands manually on the keyboard,
add the line `#define DEBUG_ON` to `main.cpp`