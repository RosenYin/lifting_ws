# usb rules

## 只有一种硬件地址的设备

```shell
#例如显示usb信息为：
lsusb
```

```shell
Bus 001 Device 010: ID 10c4:8108 Cygnal Integrated Products, Inc.
```

则`idVendor`是`10c4`，`idProduct`是`8108`

```shell
# 进入/etc/udev/rules.d目录
cd /etc/udev/rules.d
# 创建新的规则
sudo touch test.rules
# 编辑规则
sudo gedit test.rules
```

输入：

```shell
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="8108", MODE:="0777",SYMLINK+="test"
```

其中`KERNEL`判断该设备是否为ttyUSB设备，如果不是则去掉该判断;`SYMLINK`为将该usb设备映射为其他别名的设备，输入

```shell
ll /dev/test
```

可以看到映射关系

```shell
lrwxrwxrwx 1 root root 12 9月  28 09:47 /dev/ttyUSB0 -> input/event6
```

重启串口

```shell
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload-rules && sudo udevadm trigger
```

拔插设备即可。

## 有多个相同硬件地址的设备

注:**如果有两个相同或以上的idVendor,输入下列命令查看usb的KERNELS信息，作为规则的判断**

需要将usb设备绑定在工控机指定的usb端口

假设是`/dev/ttyUSB0`

### 使用KERNELS来区分

```shell
udevadm info --attribute-walk --name=/dev/ttyUSB0 | grep KERNELS
```

```shell
KERNELS=="input16"
    KERNELS=="0003:10C4:8108.0005"
    KERNELS=="1-4.4:1.0"
    KERNELS=="1-4.4"
    KERNELS=="1-4"
    KERNELS=="usb1"
    KERNELS=="0000:00:14.0"
    KERNELS=="pci0000:00"
```

如结果为上面显示的，则添加的规则判断为去掉冒号前，即 `KERNELS=="1-4.4"`  ，完整规则为：

```shell
KERNEL=="ttyUSB*",KERNELS=="1-4.4",ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="8108", MODE:="0777",SYMLINK+="test"
```

### 使用devpath来区分

根据USB在设备的序号，devpath

```shell
udevadm info --attribute-walk --name=/dev/ttyUSB0 | grep devpath
```

```shell
Udevadm info starts with the device specified by the devpath and then
    ATTRS{devpath}=="3"
    ATTRS{devpath}=="0"
```

如结果为上面显示的，则添加的规则判断为去掉冒号前，即 `ATTRS{devpath}=="3"` ，另一种完整规则为：

```shell
KERNEL=="ttyUSB*",ATTRS{devpath}=="2.1.4",ATTRS{idVendor}=="10c4",ATTRS{idProduct}=="8108",MODE:="0777",SYMLINK+="test"
```
