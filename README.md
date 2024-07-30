# RX-Standard (New)

### Summary
This project involves updating the RX chip heartbeat signal from 60s down to 5s as well as compatibility with older models and other improvements

| **Developer** | Hossam Elwahsh <br>[![GitHub](https://img.shields.io/badge/github-%23121011.svg?style=flat&logo=github&logoColor=white)](https://github.com/HossamElwahsh) [![LinkedIn](https://img.shields.io/badge/linkedin-%230077B5.svg?style=flat&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/hossam-elwahsh/) [![upwork](https://img.shields.io/badge/UpWork-6FDA44?style=flat&logo=Upwork&logoColor=white)](https://www.upwork.com/freelancers/~01656be5952e34f07d) |
|---------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------| 
| Last updated  | 28 feb 2024                                                                                                                                                                                                                                                                                                                                                                                                                                                                |


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

    | Address                                             | Data                                                       |
    |-----------------------------------------------------|------------------------------------------------------------|
    | **0x1C00** <br> might be at the<br>end of this page | Paired address of sender to me (src) - (Random TX address) |
    | **0x5000** (page 20)                                | self address                                               |
