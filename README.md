# WiFi-Buddy
WiFi Buddy was inspired by a simple insight: network knowledge should be accessible and friendly!

Most networking tools require users to buy specialized hardware, expensive licenses, and large time commitments to perform audits and surveys. Purpose-built, open source, and cheap to manufacture gets these tools in the hands of more people. 

**WiFi Buddy** is a handheld network monitoring system that continuously scans nearby wireless access points and their locations.

## Practical applications:
- Governmental: Identify infrastructure weaknesses for large scale coverage and access.

- Enterprise: Perform site surveys of operating locations for frequent cybersecurity updates.

- Consumer: Enhance connectivity by finding weak-points in home networks to enhance IoT and smart-home capabilities.

## How we built it
We built **WiFi Buddy** 
We built WIFI Buddy as a compact and simple system optimized for mobility and ease of use.

**ESP32-S3-DevkitC** was used as networks antennas and processing tools with **TFT LCD Display** provides a large, real-time readout or location of networks found by the **ESP
GY-NEO6MV2 GPS** and **MPU9520** chips allow localization of the user and estimation of signal positions.

## Challenges
We encountered a few challenges, including:
- Enclosure spacing and ventilation
- Charging and power subsystem balancing
- TFT and ESP driver conflicts

We learned that designing a handheld technology fundamentally affects design and allowances for things like parts and power management, and especially that it is worthwhile to make complicated, expensive systems more affordable and beginner-friendly.
