# LoRa Node

仅供研究LoRa使用的一个简单设备。

- `hardware/`: PCB schematics drawed by me;
- others: firmware code generated from STM32CubeMX and Claude.

包含一个0.91英寸屏幕与简易的ui，4x4的锅仔片键盘，一个UART接口预留供调试与连接上位机（待测试）使用。

供电：5v，通过stm32的USB/5v引脚连接设备。

射频部分使用e22-400m22s成品模块，内部是sx1268。额外连了一个SMA接口出来，因此可选连接SMA/IPEX天线。

e22的ant引脚附近的0Ω电阻的焊盘，如果用IPEX，则不要焊接，如果用SMA，请焊一层均匀的锡上去连接两个焊盘。也可直接修改PCB去掉那个元件，但记得确认阻抗匹配。
