# MCU + Sensors + Database Subsystem

### Owner: Matthew Greer

This subsystem acts as the **brain** of the Solar Home Lighting System.  
It collects sensor data (motion, light, voltage), controls lighting through relays,  
and communicates with the cloud database to sync data and schedules.

---

## 1. Purpose
- Read environmental data (PIR, light, voltage)
- Decide when to turn porch and foyer lights on/off
- Upload telemetry to Firebase (cloud)
- Receive config and schedule updates from cloud

---
