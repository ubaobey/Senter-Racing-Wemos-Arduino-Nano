# SENTER RACING

Firmware lampu senter berbasis **Wemos D1 mini (ESP8266)** dengan pembaca
tegangan baterai terpisah berbasis **Arduino Nano**. Wemos mengatur output LED,
kipas, layar, tombol, serta perlindungan baterai dan suhu. Nano membaca tegangan
baterai dan mengirimkannya ke Wemos melalui serial.

## Isi repositori

```text
SENTER_RACING/
├── README.md
├── SENTER_RACING_WEMOS/
│   └── SENTER_RACING_WEMOS.ino
└── NANO_BATTERY_READER/
    └── NANO_BATTERY_READER.ino
```

Kedua sketch adalah bagian dari sistem yang sama, tetapi harus diunggah ke
board masing-masing.

## Fitur

- Lima mode lampu: **LOW**, **MEDIUM**, **HIGH**, **STROBO**, dan **TURBO**.
- Pengaturan intensitas dan opsi sistem melalui tombol serta OLED.
- Pengaturan disimpan di EEPROM Wemos.
- Pemantauan tegangan baterai LiPo 3S, indikator persentase, dan pembatasan
  output saat baterai lemah.
- Pembacaan suhu DS18B20 dan pengaturan kipas.
- Komunikasi serial Wemos–Nano memakai sequence number dan checksum XOR.
- Peringatan OLED apabila Nano tidak terdeteksi.

## Perangkat keras

- Wemos D1 mini berbasis ESP8266.
- Arduino Nano.
- Baterai LiPo 3S.
- OLED SSD1306 I2C 128×64.
- Sensor suhu DS18B20.
- Kipas dan rangkaian driver LED yang sesuai dengan arus/beban LED.
- Tombol utama, tombol tambah, dan tombol kurang.
- Resistor pembagi tegangan baterai dan resistor untuk penyesuaian level serial
  seperti diagram berikut.

### Pin Wemos

| Pin Wemos | Sambungan |
| --- | --- |
| D1 | OLED SCL |
| D2 | OLED SDA |
| D3 | Tombol kurang |
| D4 | DS18B20 |
| D5 | PWM driver LED |
| D6 | Tombol utama |
| D7 | Tombol tambah |
| D0 | PWM kipas |
| RX | Terima data dari Nano D1/TX melalui pembagi level |
| TX | Kirim data ke Nano D0/RX |

### Nano dan komunikasi serial

| Pin Nano | Sambungan |
| --- | --- |
| A0 | Titik tengah pembagi tegangan baterai |
| D0/RX | Wemos TX |
| D1/TX | Wemos RX melalui resistor 1 kΩ seri dan 1,8 kΩ ke GND |
| GND | GND bersama dengan Wemos dan negatif baterai |

Skema pembagi level pada jalur **Nano TX ke Wemos RX**:

```text
Nano D1/TX --- 1 kΩ ---+--- Wemos RX
                       |
                      1,8 kΩ
                       |
                      GND
```

### Penting: nilai resistor pembagi baterai

Komentar wiring di kedua sketch menyebut **100 kΩ dan 15 kΩ**, tetapi kode
`NANO_BATTERY_READER.ino` saat ini menghitung tegangan menggunakan
`BATTERY_R1 = 100000` dan `BATTERY_R2 = 10000` (100 kΩ dan 10 kΩ). Pastikan
nilai resistor fisik cocok dengan perhitungan sebelum menyalakan rangkaian:

- Jika memakai **100 kΩ + 10 kΩ**, pertahankan nilai `BATTERY_R2` di kode Nano.
- Jika memakai **100 kΩ + 15 kΩ**, ubah `BATTERY_R2` di kode Nano menjadi
  `15000.0`.

Periksa juga referensi ADC Nano (`ADC_REFERENCE_MILLIVOLTS`, default 5000 mV)
dan kalibrasikan terhadap tegangan AVcc yang terukur. Jangan hubungkan baterai
langsung ke pin A0; pastikan tegangan pada pin ADC tidak melampaui batas board.

## Library dan persiapan

### Wemos D1 mini

Di Arduino IDE, pasang dukungan board **ESP8266** dan library berikut:

- Adafruit GFX Library
- Adafruit SSD1306
- OneWire
- DallasTemperature

`Wire`, `EEPROM`, dan `ESP8266WiFi` disediakan oleh dukungan board ESP8266.

### Arduino Nano

Pilih dukungan board **Arduino AVR Boards**. Sketch Nano menggunakan `stdio.h`
dan fasilitas serial/ADC Arduino; tidak memerlukan library eksternal tambahan.

## Upload firmware

1. Buka `SENTER_RACING_WEMOS/SENTER_RACING_WEMOS.ino` di Arduino IDE.
2. Pilih board **LOLIN (WEMOS) D1 mini** (atau board ESP8266 yang sesuai),
   port COM yang benar, lalu unggah.
3. Untuk mengunggah ke Nano, buka
   `NANO_BATTERY_READER/NANO_BATTERY_READER.ino`, pilih **Arduino Nano** dan
   prosesor/bootloader yang sesuai dengan board, lalu unggah.
4. Putuskan sambungan serial antar-board saat mengunggah: lepaskan Nano TX dari
   Wemos RX ketika mengunggah ke Wemos; lepaskan Wemos TX dari Nano RX ketika
   mengunggah ke Nano.
5. Setelah upload, sambungkan kembali TX/RX sesuai tabel, pastikan GND bersama,
   lalu nyalakan sistem.

## Kontrol tombol

- Klik tombol utama 1–5 kali untuk memilih mode **LOW**, **MEDIUM**, **HIGH**,
  **STROBO**, atau **TURBO**.
- Saat lampu menyala, tekan singkat tombol utama untuk mematikan lampu.
- Saat lampu menyala, tahan tombol utama untuk berpindah di antara LOW, MEDIUM,
  dan HIGH.
- Tombol tambah/kurang mengatur target output sesuai mode; rincian parameter
  sistem dapat diakses melalui menu pengaturan pada perangkat.

## Protokol serial Nano–Wemos

Serial berjalan pada **9600 baud**. Wemos mengirim baris:

```text
Q,<sequence>,<checksum>
```

Nano membalas:

```text
R,<sequence>,<millivolt>,<checksum>
```

Setiap baris diakhiri newline. Checksum adalah XOR dari semua byte sebelum koma
terakhir. Wemos meminta data baterai setiap 250 ms.

## Catatan keselamatan

- Pastikan polaritas baterai, pembagi tegangan, GND bersama, dan level logika
  telah diperiksa sebelum memberi daya.
- Jangan menggerakkan LED berarus tinggi langsung dari pin Wemos; gunakan
  MOSFET/driver yang sesuai dan pendinginan yang memadai.
- Lepaskan baterai saat melakukan perubahan wiring atau pemrograman.
- Uji pembacaan tegangan dengan multimeter sebelum mengandalkan fitur proteksi
  baterai.
