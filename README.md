# STM32F051_StdPeriph_Lib_V1.4.0_IAP_VarAllocateII

update @ 2019/04/12

Add variable allocate example in KEILC

search AssignedVar1 in Bootloader sample code 

Scenario : 

1. In Boot loader flow , check the data in KEY1_Address , to decide update firmware or jump to application

2. In application flow , revise the data in KEY1_Address , to decide update firmware in next MCU reset stage