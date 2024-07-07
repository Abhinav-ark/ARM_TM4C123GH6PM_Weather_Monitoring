<h1 align='center'>TM4C123GH6PM Weather Monitoring System</h1>

<div align='center'>
  <img src='./Assets/cover.jpeg' width='600px'/>
</div>

A comprehensive weather monitoring system that utilizes `ADC` for efficient sensor integration, `I2C` and  `OLED` for RealTime Data Monitoring, and incorporates `Bluetooth(UART)` technology to enable wireless transmission of weather Data, fostering a reliable and user-friendly solution for real-time weather data acquisition and analysis.

## Motivation
There is a heavy demand for remote realtime Temperature and Humidity monitoring systems in multiple fields. <br>
1. <b>Medical Instruments:</b> Certain medical instruments need to be stored at specific temperatures and humidity levels. <br>
2. <b>Food Processing:</b> Maintaining the right temperature and humidity is crucial for food safety and quality. <br>
3. <b>Warehouses and Cold Storages:</b> Warehouses storing temperature-sensitive products like chemicals, fruits, vegetables, food, medicines, etc., need these systems. <br>
4. <b>Agriculture:</b> Monitoring temperature and humidity can help optimize crop growth and prevent disease. <br>
5. <b>Data Centers:</b> These facilities require strict climate control to prevent overheating and ensure optimal performance.


## Circuit Diagram

<div align='center'>
  <img src='./Assets/circuit_diagram.png' width='600px'/>
</div>

## Features
### I2C Sensor Integration
Utilizing I2C facilitates the integration of diverse sensors, such as temperature, humidity, and atmospheric pressure, providing comprehensive weather data.
### Bluetooth Connectivity
Incorporating Bluetooth technology enables wireless communication, allowing users to access and control the system remotely, fostering increased flexibility and convenience.
### Real-time Data
The system provides real-time weather data, allowing for timely decision-making and analysis.
### User-friendly Interface
A user-friendly interface enhances accessibility, making it easier for users to interact with the system, visualize data, and configure settings.
### Scalability
The modular design allows for the addition of new sensors or features, making the system adaptable to evolving weather monitoring needs.

## Working Demo
https://github.com/Abhinav-ark/ARM_TM4C123GH6PM_Weather_Monitoring/assets/76396917/d2604c1a-f7d5-4534-a5a2-c1766f69a376

## Instructions to setup

### Hardware and Source Code
- Make the Circuit Connections as per the Circuit Diagram.
- Create a new Tiva `TM4C123GH6PM` Project in `Keil μVision-5`.
- Select the following checkboxes in the Manage Run-Time Environment Window.
<div align='center'>
  <img src='./Assets/keil_project_conf.png' width='600px'/>
</div>  

- Copy the source code file `main.c` to your Keil Project under `Target` > `Source Group`.
- Build the project (F7) and Set the Options for Target as follows
<div align='center'>
  <img src='./Assets/target_options.png' width='500px'/>
</div>  

- Download the code into the Microcontroller's Flash Memory (F8) and press the Reset button on the Microcontroller to run the code.

### Interacting with the Hardware Using Bluetooth 
- Install the [`Serial Bluetooth Terminal`](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&hl=en_IN) App from Play Store on your Mobile Device.
- After that turn on Bluetooth of your smartphone or Android based mobile and scan for available Bluetooth devices. You will see the HC-05 device in your scanned list. The default name for this Bluetooth device is <b>“HC-05”</b> and the default pin code is either <b>“0000”</b> or <b>“1234”</b>.
- Now open your serial Bluetooth terminal Android application and click on menu->devices. You will find the list of available nearby Bluetooth devices.
- Select HC-05 and the serial Bluetooth terminal starts to pair with HC-05. It will ask for the pin code and enter the pin code mentioned above.
- After pairing, it will show a successful connection message.
- 1. Now Type 'T' and click on the send button to get the current temperature recorded by the device.
  2. Type 'H' and click on the send button to get the current humidity recorded by the device.
 
# LICENSING
All code in this repository is Licensed Under [`MIT license`](https://github.com/Abhinav-ark/ARM_TM4C123GH6PM_Weather_Monitoring/blob/main/LICENSE), and is freely available for anyone to use or make changes.

# Security Policy
Refer [`SECURITY.md`](https://github.com/Abhinav-ark/ARM_TM4C123GH6PM_Weather_Monitoring/blob/main/SECURITY.md) for Security Policy.

