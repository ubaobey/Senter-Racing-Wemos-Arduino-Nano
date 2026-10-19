# SENTER RACING — Skema Hardware

Dokumen ini menjelaskan sambungan hardware untuk firmware Wemos D1 Mini dan
Arduino Nano battery reader.

## Komponen

- Wemos D1 Mini / ESP8266
- Arduino Nano (logika 5 V)
- Baterai LiPo 3S (maksimum 12,6 V)
- OLED SSD1306 I2C, 128 × 64, alamat I2C `0x3C`
- Sensor suhu DS18B20 dan resistor pull-up 4,7 kΩ
- Tiga tombol tekan: MAIN, PLUS, dan MINUS
- Beban LED dan kipas, masing-masing dengan driver yang sesuai
- Pembagi tegangan baterai: 100 kΩ dan 10 kΩ
- Pembagi level serial: 1 kΩ dan 1,8 kΩ
- Regulator DC-DC 5 V yang sesuai untuk arus total rangkaian

## Skema koneksi

```text
BATTERY / POWER
==============
LiPo 3S BAT+ (9–12,6 V) --+----> Driver LED / kipas, sesuai rating beban*
                           +----> IN+ regulator DC-DC 5 V
                           +-- 100 kΩ --+----> Nano A0
                                        |
                                       10 kΩ
                                        |
LiPo 3S BAT- ---------------------------+---- GND BERSAMA
                                              +-- Nano GND
                                              +-- Wemos GND
                                              +-- driver GND
                                              +-- sensor / OLED GND

Regulator OUT+ 5 V ---------------------------+-- Nano 5V
                                              +-- Wemos 5V/VIN**
OLED VCC ---------------- Wemos 3V3
DS18B20 VDD ------------ Wemos 3V3

WEMOS D1 MINI / ESP8266
=======================
D1 / GPIO5  ---------------- OLED SCL
D2 / GPIO4  ---------------- OLED SDA
D3 / GPIO0  ---------------- Tombol MINUS ---- GND
D4 / GPIO2  ---------------- DS18B20 DATA
                              |
                             4,7 kΩ
                              |
                             3V3
D5 / GPIO14 PWM ------------ Driver LED (input PWM)
D6 / GPIO12 --------------- Tombol MAIN ------ GND
D7 / GPIO13 --------------- Tombol PLUS ------ GND
D0 / GPIO16 PWM ------------ Driver kipas (input PWM)
D8 / GPIO15 ---------------- Tidak digunakan
TX / GPIO1 ----------------- Nano D0 / RX
RX / GPIO3 <----------------+---- 1 kΩ ---- Nano D1 / TX
                            |
                           1,8 kΩ
                            |
                           GND
```

`*` Daya untuk LED dan kipas mengikuti spesifikasi bebannya dan driver yang
digunakan. Jangan mengalirkan arus beban langsung melalui pin Wemos. Gunakan
driver LED yang menerima PWM 3,3 V dan driver kipas yang sesuai dengan jenis
kipasnya; satukan ground driver dengan ground Wemos. Untuk kipas DC dua-kawat
yang dikendalikan dengan MOSFET, gunakan MOSFET logic-level dan proteksi
induktif yang sesuai. Jangan gunakan rangkaian low-side tersebut untuk kipas
empat-kawat yang memiliki masukan PWM khusus; ikuti spesifikasi kipasnya.

`**` Beri daya board dari regulator 5 V yang stabil melalui pin 5V/VIN sesuai
revisi board. Jangan hubungkan keluaran 5 V regulator ke pin 3V3. Hindari
menyuplai board dari USB dan pin 5V/VIN sekaligus kecuali rangkaian board
memang mendukungnya.

## Tabel pin Wemos

| Pin Wemos | Sambungan | Catatan |
| --- | --- | --- |
| D1 / GPIO5 | OLED SCL | I2C |
| D2 / GPIO4 | OLED SDA | I2C; OLED diberi daya 3,3 V agar level I2C cocok |
| D3 / GPIO0 | Tombol MINUS ke GND | Firmware memakai `INPUT_PULLUP` |
| D4 / GPIO2 | DS18B20 DATA | Tambahkan pull-up 4,7 kΩ dari DATA ke 3V3 |
| D5 / GPIO14 | Masukan PWM driver LED | PWM 0–3,3 V; bukan untuk LED berdaya langsung |
| D6 / GPIO12 | Tombol MAIN ke GND | Firmware memakai `INPUT_PULLUP` |
| D7 / GPIO13 | Tombol PLUS ke GND | Firmware memakai `INPUT_PULLUP` |
| D0 / GPIO16 | Masukan PWM driver kipas | Gunakan driver/antarmuka yang sesuai dengan kipas |
| D8 / GPIO15 | Tidak digunakan | |
| TX / GPIO1 | Nano D0 / RX | UART 9600 baud, Wemos TX 3,3 V |
| RX / GPIO3 | Nano D1 / TX melalui pembagi | Jangan masukkan sinyal UART 5 V langsung ke RX Wemos |

## Pembaca baterai dan pembagi tegangan

Hubungkan baterai positif ke Nano A0 melalui resistor seri 100 kΩ. Hubungkan
A0 ke ground melalui resistor 10 kΩ. Baterai negatif, Nano GND, dan Wemos GND
harus menjadi satu ground bersama.

```text
Battery + ---- 100 kΩ ----+---- Nano A0
                          |
                         10 kΩ
                          |
Battery - ----------------+---- GND bersama
```

Dengan nilai yang dipakai oleh program Nano, tegangan pada A0 sekitar 1,15 V
saat baterai 12,6 V. Jangan sambungkan baterai 3S langsung ke A0. Pastikan
tegangan A0 tidak melewati batas ADC board Nano.

**Penting:** konstanta program Nano menetapkan `R1 = 100 kΩ` dan
`R2 = 10 kΩ`. Komentar hardware pada header kedua sketch masih menyebut 15 kΩ;
nilai pada komentar itu sudah tidak cocok dengan perhitungan firmware. Untuk
firmware saat ini, gunakan 10 kΩ sebagai resistor dari A0 ke GND. Nilai
resistor pembagi level UART (1 kΩ dan 1,8 kΩ) adalah rangkaian terpisah dan
tidak boleh tertukar dengan pembagi baterai.

## Sambungan UART Nano–Wemos

- Wemos TX ke Nano D0/RX.
- Nano D1/TX ke Wemos RX melalui pembagi 1 kΩ seri dan 1,8 kΩ dari titik RX
  Wemos ke GND.
- Hubungkan GND kedua board.
- Kedua sketch menggunakan UART 9600 baud.
- Saat mengunggah firmware ke Nano, lepaskan jalur Wemos TX ke Nano RX.
- Saat mengunggah firmware ke Wemos, lepaskan jalur Nano TX/pembagi ke Wemos RX.

## Catatan perakitan

- Ketiga tombol dipasang antara pin GPIO masing-masing dan GND; tidak perlu
  resistor pull-up eksternal karena firmware mengaktifkan `INPUT_PULLUP`.
- DS18B20 memerlukan pull-up 4,7 kΩ antara DATA dan 3V3.
- OLED memakai I2C pada alamat `0x3C`; suplai 3,3 V menghindari pull-up I2C
  5 V yang dapat merusak Wemos.
- Satukan ground catu daya, Wemos, Nano, sensor, dan driver agar sinyal UART
  serta PWM memiliki referensi yang sama.
- Pastikan regulator dan kabel mampu menangani arus beban LED/kipas. Jangan
  mengalirkan arus LED atau kipas melalui pin GPIO maupun regulator onboard
  Wemos/Nano.
- Ukur tegangan baterai di A0 sebelum menghubungkan baterai untuk pertama kali.

## Berkas firmware

- [Firmware Arduino Nano](./NANO_BATTERY_READER.ino)
- [Firmware Wemos D1 Mini](./SENTER_RACING_WEMOS/SENTER_RACING_WEMOS.ino)
