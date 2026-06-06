# Pinout — Waveshare ESP32-S3-Touch-LCD-2.8C (480×480 rund, ST7701 RGB)

> ✅ **Verifiziert gegen ESPHome-Community-Config** (HA-Forum, 2.8C-spezifisch).
> Restrisiko: Expander-Typ (PCA9554 vs TCA9554 — pinkompatibel) und GT911
> I2C-Adresse final am Geraet pruefen.

## LCD ST7701 — RGB-Interface

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| HSYNC | 38 | | VSYNC | 39 |
| PCLK | 41 | | DE | 40 |
| BL (Backlight) | 6 | | | |

### RGB-Datenpins (RGB565) — 5R / 6G / 5B

| Rot | GPIO | Gruen | GPIO | Blau | GPIO |
|---|---|---|---|---|---|
| R0 | 46 | G0 | 14 | B0 | 5 |
| R1 | 3 | G1 | 13 | B1 | 45 |
| R2 | 8 | G2 | 12 | B2 | 48 |
| R3 | 18 | G3 | 11 | B3 | 47 |
| R4 | 17 | G4 | 10 | B4 | 21 |
| | | G5 | 9 | | |

> Korrektur ggü. 2.1"-Variante: Blau hat bei der 2.8C **5 Pins** (B4 = GPIO21).

### ST7701 SPI-Init (nur fuer Init-Sequenz)

| Signal | Anschluss |
|---|---|
| SCK | GPIO2 |
| MOSI/SDA | GPIO1 |
| CS | **Expander Pin 2** |
| RST | **Expander Pin 0** |

## Touch GT911 — I2C

| Signal | GPIO | Hinweis |
|---|---|---|
| SDA | 15 | |
| SCL | 7 | |
| INT | 16 | |
| RST | **Expander Pin 1** | |
| I2C-Adresse | 0x5D (alt) / 0x14 | GT911 typisch |

## PCA9554 I2C-GPIO-Expander (@ 0x20)

0-basierte Pin-Nummern (ESPHome-Konvention):

| Expander-Pin | Funktion |
|---|---|
| 0 | Display-Reset (ST7701 RST) |
| 1 | Touch-Reset (GT911 RST) |
| 2 | LCD Chip-Select (ST7701 CS) |

Muss vor LCD-Init initialisiert werden: alle drei auf HIGH, dann
Reset-Sequenz (LOW→HIGH) fuer Display und Touch.

## Quelle

ESPHome Community-Config Waveshare ESP32-S3-Touch-LCD-2.8C
(home-assistant.io Forum) + LovyanGFX Discussion #630 (2.1" Referenz).
