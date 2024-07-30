# RX-Standard (New) - Supports Magnetic Switch

### Summary
This project involves updating the RX chip heartbeat signal from 60s down to 5s as well as compatibility with older models and other improvements

| **Developer** | Hossam Elwahsh <br>[![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/HossamElwahsh) [![LinkedIn](https://img.shields.io/badge/linkedin-%230077B5.svg?style=flat&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/hossam-elwahsh/) [![upwork](https://img.shields.io/badge/UpWork-6FDA44?style=flat&logo=Upwork&logoColor=white)](https://www.upwork.com/freelancers/~01656be5952e34f07d) |
|---------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------| 
| Last updated  | 30 jul 2024                                                                                                                                                                                                                                                                                                                                                                                                                                                                |


---- 

### Compilation
Just open and compile using `IAR IDE for 8051` normally
                                                        
---

## Configurations
>    All the following configs are in file `RxBoardDef.h`
   
  | Config                       | Usage                                                                                                         |
  |------------------------------|---------------------------------------------------------------------------------------------------------------|
  | **MY_MAC_ADDRESS**           | Defines RF address for self                                                                                   |
  | **NOTIFIER_TAG_MAC_ADDRESS** | Defines RF address for receiver of the heartbeat signal (notifier) <br/><br/> Constant value >>`0x98765432`<< |


### Magnetic Switch Support
Configurations for this can be found in `app_magnetic_cfg.h`

### `MAGNETIC_PCB_TYPE` selects PCB type, options are:
- `MAGNETIC_PCB_TYPE_OLD_OPT` : magnetic sensor chip is connected to P0 Pin 1
- `MAGNETIC_PCB_TYPE_NEW_OPT` : magnetic sensor chip is connected to P0 Pin 7


### `PROJECT_TYPE_CFG` selects project type, options are:
- `PROJECT_TYPE_BASE_OPT` : normal new RX project
- `PROJECT_TYPE_RX_MINI_4_IN_MAGNETIC_PROXIMITY_OPT` : magnetic sensor chip, with 1 output called (BRIGHT_CTRL)

### How to switch
- during the first `15s` (`TIME_ALLOWED_MAGNETIC_CHANGE_10MS`) switch can be made by holding a magnet to the board for more than `5s` or (`TIME_HOLD_REQ_FOR_MAGNETIC_CHANGE_10MS`)


---

## Notes
- ### Pairing
  - Flexible updates to pair address within `TIME_PAIRING_ALLOWED_10MS` from startup (default is 15s).
  - Pairing is turned off if a signal is received from a previously paired device is detected.
  - Upon the first successful pair _(by sending left signal)_ the address of the paired device is saved on the flash, and will continue to be used for subsequent power ons

- ### Output & Repeated Signal - 1 RF Byte
    - Repeated as received
    - Output on Port.0

- ### Flash

    | Address                                             | Data                                                                                               |
    |-----------------------------------------------------|----------------------------------------------------------------------------------------------------|
    | **0x1C00** <br> might be at the<br>end of this page | Paired address of sender to me (src) - (Random TX address)                                         |
    | **0x5000** (page 20)                                | self address                                                                                       |
    | **0x5400** (page 21)                                | app data, first byte: current magnetic app follow mode, (follow right signal / follow left signal) |
