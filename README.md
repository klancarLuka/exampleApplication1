# Irnas Coding Challenge

This repository contains a Zephyr coding challenge application that extends the example application repository ([link][example]).

This app demonstrates the use of the bluetooth low-power technology, ment for the STM32WB55rg Nucleo board.

The main source file is contained in the app folder.

### Getting Started & Initialization

Before initialization, make sure you have a proper Zephyr development
environment. You can follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

After acquiring a proper Zephyr development environment, you have to clone all modules needed by Zephyr. You can do that by running:

```shell
# initialize my-workspace for the application (main branch)
west init -m https://github.com/klancarLuka/exampleApplication1.git --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```
### Building & running
You can build the application by running 
```shell
west build -p always -b nucleo_wb55rg app
```
Finnaly you have to flash the program to the bord. That can be accieved by running the 
```shell
west flash
```
or by using creating a manual instalation of west and using `ninja`:
```shell
ninja flash
```
First clone `west`:
```shell
git clone https://github.com/zephyrproject-rtos/west.git .west/west
```
then create a `.west/config` file with the following contents:
```shell
[manifest]
path = zephyr

[zephyr]
base = zephyr
```
Now we just have to let `ninja` know the location of `west`.
We can do that by setting the `WEST_DIR` environment variable, so that it points to [zephyrproject/.west/west]
- 

