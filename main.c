/*
 * Project: Bitmain Antminer S19 Series PIC16F1704 Firmware Patch-Fix (Final Fixed Interrupts)
 * Author: Anton Vinogradov @vinantole support@minerflash.ru
 * Year: 2026
 * 
 * Description:
 * Исправленная версия. Адрес определяется пинами RC3-RC5 при старте.
 * Использует ПРЕРЫВАНИЯ для быстрой работы с тестером и КП.
 * Команда 0x07 выполняет SOFT RESET (сброс состояния без перезагрузки MCU).
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 32000000UL

#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF, BOREN = ON
#pragma config CLKOUTEN = OFF, IESO = ON, FCMEN = ON, WRT = OFF, PPS1WAY = ON, PLLEN = ON
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

// Базовый адрес I2C
#define I2C_BASE_ADDRESS_MIN    0x20
#define MAX_BUFFER_SIZE         16
#define FW_VERSION              0xAB

/* Адреса датчиков температуры TMP75 */
#define TMP75_ADDR_0        0x48
#define TMP75_ADDR_1        0x49
#define TMP75_ADDR_2        0x4A
#define TMP75_ADDR_3        0x4B
#define TMP75_REG_TEMP      0x00

/* Пины BitBang I2C для датчиков */
#define TWI_SDA_TRIS        TRISAbits.TRISA4
#define TWI_SDA_LAT         LATAbits.LATA4
#define TWI_SDA_PORT        PORTAbits.RA4
#define TWI_SCL_TRIS        TRISAbits.TRISA5
#define TWI_SCL_LAT         LATAbits.LATA5
#define TWI_DELAY()         __delay_us(2)

/* Пин управления ключами питания чипов на хеш-плате */
#define POWER_KEY_LAT       LATAbits.LATA2
#define POWER_KEY_TRIS      TRISAbits.TRISA2
#define POWER_KEY_ANSEL     ANSELAbits.ANSA2

typedef enum {
    RX_STATE_IDLE,
    RX_STATE_PREAMBLE,
    RX_STATE_LENGTH,
    RX_STATE_DATA
} RX_State;

volatile RX_State rx_state = RX_STATE_IDLE;
volatile uint8_t rx_buffer[MAX_BUFFER_SIZE];
volatile uint8_t rx_idx = 0;
volatile uint8_t packet_len = 0;
volatile uint8_t tx_buffer[MAX_BUFFER_SIZE];
volatile uint8_t tx_len = 0;
volatile uint8_t tx_idx = 0;
volatile bool reset_pending = false;
volatile bool packet_ready = false; // Флаг готовности пакета для главного цикла

void process_packet(void);
uint16_t adc_read_channel(uint8_t channel);
bool tmp75_read_with_fallback(uint8_t primary_addr, uint8_t *temp_int, uint8_t *temp_frac);
bool twi_bitbang_write_with_fallback(uint8_t target_addr, uint8_t *data, uint8_t len);
void twi_bitbang_init_pins(void);
void twi_bitbang_start(void);
void twi_bitbang_stop(void);
bool twi_bitbang_write_byte(uint8_t data);
uint8_t twi_bitbang_read_byte(bool ack);
void I2C_Slave_Init(uint8_t addr);

/**
Проверка дополнительных байтов пакета на равенство нулю.
*/
bool check_extra_bytes_zero(void) {
    for (uint8_t i = 2; i < packet_len - 1; i++) {
        if (rx_buffer[i] != 0x00) {
            return false;
        }
    }
    return true;
}

// --- BitBang I2C Functions ---

void twi_bitbang_init_pins(void) {
    ANSELAbits.ANSA4 = 0;
    TWI_SDA_TRIS = 1; TWI_SCL_TRIS = 1;
    TWI_SDA_LAT = 0; TWI_SCL_LAT = 0;
}

void twi_bitbang_start(void) {
    TWI_SDA_TRIS = 1; TWI_SCL_TRIS = 1; TWI_DELAY();
    TWI_SDA_TRIS = 0; TWI_DELAY();
    TWI_SCL_TRIS = 0; TWI_DELAY();
}

void twi_bitbang_stop(void) {
    TWI_SDA_TRIS = 0; TWI_DELAY();
    TWI_SCL_TRIS = 1; TWI_DELAY();
    TWI_SDA_TRIS = 1; TWI_DELAY();
}

bool twi_bitbang_write_byte(uint8_t data) {
    uint8_t i; bool ack;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) TWI_SDA_TRIS = 1; else TWI_SDA_TRIS = 0;
        TWI_DELAY(); TWI_SCL_TRIS = 1; TWI_DELAY(); TWI_SCL_TRIS = 0;
        data <<= 1;
    }
    TWI_SDA_TRIS = 1; TWI_DELAY(); TWI_SCL_TRIS = 1; TWI_DELAY();
    ack = !TWI_SDA_PORT;
    TWI_SCL_TRIS = 0; TWI_DELAY();
    return ack;
}

uint8_t twi_bitbang_read_byte(bool ack) {
    uint8_t i, data = 0;
    TWI_SDA_TRIS = 1;
    for (i = 0; i < 8; i++) {
        data <<= 1; TWI_SCL_TRIS = 1; TWI_DELAY();
        if (TWI_SDA_PORT) data |= 0x01;
        TWI_SCL_TRIS = 0; TWI_DELAY();
    }
    if (ack) TWI_SDA_TRIS = 0; else TWI_SDA_TRIS = 1;
    TWI_DELAY(); TWI_SCL_TRIS = 1; TWI_DELAY(); TWI_SCL_TRIS = 0; TWI_DELAY();
    TWI_SDA_TRIS = 1;
    return data;
}

bool tmp75_read_with_fallback(uint8_t primary_addr, uint8_t *temp_int, uint8_t *temp_frac) {
    twi_bitbang_init_pins();
    twi_bitbang_start();
    if (twi_bitbang_write_byte((uint8_t)(primary_addr << 1)) &&
        twi_bitbang_write_byte(TMP75_REG_TEMP)) {
        twi_bitbang_start();
        if (twi_bitbang_write_byte((uint8_t)((primary_addr << 1) | 0x01))) {
            uint8_t msb = twi_bitbang_read_byte(true);
            uint8_t lsb = twi_bitbang_read_byte(false);
            twi_bitbang_stop();
            uint16_t temp_raw = ((uint16_t)msb << 4) | (lsb >> 4);
            *temp_int = (uint8_t)(temp_raw >> 4);
            *temp_frac = (uint8_t)(temp_raw & 0x0F);
            return true;
        }
    }
    twi_bitbang_stop();
    if (primary_addr != TMP75_ADDR_0) {
        twi_bitbang_start();
        if (twi_bitbang_write_byte((uint8_t)(TMP75_ADDR_0 << 1)) &&
            twi_bitbang_write_byte(TMP75_REG_TEMP)) {
            twi_bitbang_start();
            if (twi_bitbang_write_byte((uint8_t)((TMP75_ADDR_0 << 1) | 0x01))) {
                uint8_t msb = twi_bitbang_read_byte(true);
                uint8_t lsb = twi_bitbang_read_byte(false);
                twi_bitbang_stop();
                uint16_t temp_raw = ((uint16_t)msb << 4) | (lsb >> 4);
                *temp_int = (uint8_t)(temp_raw >> 4);
                *temp_frac = (uint8_t)(temp_raw & 0x0F);
                return true;
            }
        }
        twi_bitbang_stop();
    }
    return false;
}

bool twi_bitbang_write_with_fallback(uint8_t target_addr, uint8_t *data, uint8_t len) {
    uint8_t addrs[] = {TMP75_ADDR_0, TMP75_ADDR_1, TMP75_ADDR_2, TMP75_ADDR_3};
    bool success = false;
    twi_bitbang_init_pins();
    twi_bitbang_start();
    if (twi_bitbang_write_byte((uint8_t)(target_addr << 1))) {
        success = true;
        for (uint8_t i = 0; i < len; i++) twi_bitbang_write_byte(data[i]);
    }
    twi_bitbang_stop();
    if (!success) {
        for (uint8_t k = 0; k < 4; k++) {
            if (addrs[k] == target_addr) continue;
            twi_bitbang_init_pins();
            twi_bitbang_start();
            if (twi_bitbang_write_byte((uint8_t)(addrs[k] << 1))) {
                success = true;
                for (uint8_t i = 0; i < len; i++) twi_bitbang_write_byte(data[i]);
                twi_bitbang_stop();
                break;
            }
            twi_bitbang_stop();
        }
    }
    return success;
}

uint16_t adc_read_channel(uint8_t channel) {
    ADCON1 = 0xD0;
    ADCON0 = (uint8_t)((channel << 2) | 0x01);
    __delay_us(10);
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE);
    return ((uint16_t)ADRESH << 8) | ADRESL;
}

/**
 * Инициализация I2C Slave с включенными ПРЕРЫВАНИЯМИ.
 */
void I2C_Slave_Init(uint8_t addr) {
    ANSELC = 0x00;
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;
    
    SSPCLKPPS = 0x10;
    SSPDATPPS = 0x11;
    RC0PPS = 0x10;
    RC1PPS = 0x11;
    
    SSPADD = (uint8_t)(addr << 1);
    SSPCON1 = 0x36;   // I2C Slave, 7-bit, Enable
    SSPCON2 = 0x01;   // Clock Stretching enabled
    SSPCON3 = 0x18;   // BOEN + SDAHT (Критично для скорости!)
    
    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 1; // Разрешаем прерывания от MSSP
    INTCONbits.PEIE = 1; // Разрешаем периферийные прерывания
    INTCONbits.GIE = 1;  // Глобальное разрешение прерываний
}

/**
 * Обработчик прерывания MSSP (I2C).
 * Работает максимально быстро, просто сохраняя данные в буфер.
 */
void __interrupt() ISR_MSSP(void) {
    if (PIR1bits.SSP1IF) {
        uint8_t dummy;
        
        // 1. Обработка Write Collision
        if (SSPCON1bits.WCOL) {
            SSPCON1bits.WCOL = 0;
            dummy = SSPBUF;
            rx_state = RX_STATE_IDLE;
        }
        // 2. Мастер читает данные (Read)
        else if (SSPSTATbits.R_nW) {
            if (SSPSTATbits.BF && !SSPSTATbits.D_nA) {
                dummy = SSPBUF; 
                if (tx_idx < tx_len) SSPBUF = tx_buffer[tx_idx]; else SSPBUF = 0xFF;
            } else {
                tx_idx++;
                if (tx_idx < tx_len) SSPBUF = tx_buffer[tx_idx]; else SSPBUF = 0xFF;
            }
        }
        // 3. Мастер пишет данные (Write)
        else if (SSPSTATbits.BF) {
            dummy = SSPBUF;
            if (SSPSTATbits.D_nA) { 
                switch (rx_state) {
                    case RX_STATE_IDLE:
                        if (dummy == 0x55) rx_state = RX_STATE_PREAMBLE;
                        break;
                    case RX_STATE_PREAMBLE:
                        if (dummy == 0xAA) {
                            rx_state = RX_STATE_LENGTH;
                            rx_idx = 0;
                            packet_len = 0;
                        } else {
                            rx_state = RX_STATE_IDLE;
                        }
                        break;
                    case RX_STATE_LENGTH:
                        packet_len = dummy;
                        rx_buffer[rx_idx++] = dummy;
                        if (packet_len < 2 || packet_len > MAX_BUFFER_SIZE) {
                            rx_state = RX_STATE_IDLE;
                        } else {
                            rx_state = RX_STATE_DATA;
                        }
                        break;
                    case RX_STATE_DATA:
                        if (rx_idx < MAX_BUFFER_SIZE) rx_buffer[rx_idx++] = dummy;
                        if (rx_idx >= packet_len) {
                            packet_ready = true; 
                            rx_state = RX_STATE_IDLE;
                        }
                        break;
                }
            }
        }
        
        SSPCON1bits.CKP = 1; 
        PIR1bits.SSP1IF = 0; 
    }
}

/**
 * Обработка принятого пакета (вызывается в главном цикле).
 */
void process_packet(void) {
    /* Проверка контрольной суммы */
    uint8_t calculated_checksum = 0;
    for (uint8_t i = 0; i < (packet_len - 1); i++) calculated_checksum += rx_buffer[i];
    if (calculated_checksum != rx_buffer[packet_len - 1]) {
        tx_len = 0;
        return;
    }
    
    uint8_t cmd = rx_buffer[1];
    reset_pending = false;
    
    /* --- Команды 0x01..0x05,0x08,0x09: Flash-заглушки --- */
    if (cmd >= 0x01 && cmd <= 0x02) {
        tx_buffer[0] = cmd; tx_buffer[1] = 0x00; tx_len = 2;
    }     
    else if (cmd >= 0x03 && cmd <= 0x05) {
        if (!check_extra_bytes_zero()) { tx_buffer[0] = 0xFF; tx_buffer[1] = 0xFF; tx_len = 2; }
        else { tx_buffer[0] = cmd; tx_buffer[1] = 0x00; tx_len = 2; }
    }
    else if (cmd == 0x08) {
        if (!check_extra_bytes_zero()) { tx_buffer[0] = 0xFF; tx_buffer[1] = 0xFF; tx_len = 2; }
        else { tx_buffer[0] = 0x08; tx_buffer[1] = 0x00; tx_len = 2; }
    } 
    else if (cmd == 0x09) {
        if (!check_extra_bytes_zero()) { tx_buffer[0] = 0xFF; tx_buffer[1] = 0xFF; tx_len = 2; }
        else { tx_buffer[0] = 0x09; tx_buffer[1] = 0x00; tx_len = 2; }
    } 
    
    /* --- Команда 0x06: JUMP_TO_APP --- */
    else if (cmd == 0x06) {
        if (!check_extra_bytes_zero()) { tx_buffer[0] = 0xFF; tx_buffer[1] = 0xFF; tx_len = 2; }
        else {
            tx_buffer[0] = 0x06; tx_buffer[1] = 0x01; tx_len = 2;
        }
    }

    /* --- Команда 0x07: RESET_PIC (SOFT RESET) --- */
    else if (cmd == 0x07) {
        if (!check_extra_bytes_zero()) { tx_buffer[0] = 0xFF; tx_buffer[1] = 0xFF; tx_len = 2; }
        else {
            tx_buffer[0] = 0x07; tx_buffer[1] = 0x01; tx_len = 2;
            reset_pending = true;
        }
    }

    /* --- Команда 0x15: ENABLE_VOLTAGE --- */
    else if (cmd == 0x15) {
        if (packet_len >= 3) {
            uint8_t state = rx_buffer[2];
            POWER_KEY_LAT = (state == 0x01) ? 1 : 0;
            tx_buffer[0] = 0x15; tx_buffer[1] = 0x01; tx_len = 2;
        } else { tx_len = 0; }
    }
    /* --- Команда 0x16: SEND_HEART_BEAT --- */
    else if (cmd == 0x16) {
        uint8_t status_byte = 0x00;
        tx_buffer[0] = 0x06; tx_buffer[1] = 0x16; tx_buffer[2] = 0x01; tx_buffer[3] = status_byte;
        uint16_t resp_sum = 0;
        for (uint8_t i = 0; i < 4; i++) resp_sum += tx_buffer[i];
        tx_buffer[4] = (uint8_t)(resp_sum >> 8); tx_buffer[5] = (uint8_t)(resp_sum & 0xFF);
        tx_len = 6;
    }
    /* --- Команда 0x17: GET_PIC_SOFTWARE_VERSION --- */
    else if (cmd == 0x17) {
        if (!check_extra_bytes_zero()) { tx_buffer[0] = 0xFF; tx_buffer[1] = 0xFF; tx_len = 2; }
        else {
            tx_buffer[0] = 0x05; tx_buffer[1] = 0x17; tx_buffer[2] = FW_VERSION; tx_buffer[3] = 0x00;
            tx_buffer[4] = 0xC7; // Checksum: 05+17+AB+00 = C7
            tx_len = 5;
        }
    }
    /* --- Команда 0x3A: Чтение ADC --- */
    else if (cmd == 0x3A) {
        if (!check_extra_bytes_zero()) { tx_buffer[0] = 0xFF; tx_buffer[1] = 0xFF; tx_len = 2; }
        else if (packet_len >= 3) {
            uint8_t channel = rx_buffer[2];
            if (channel == 0x00) {
                uint16_t adc_val = adc_read_channel(0x06);
                tx_buffer[0] = 0x07; tx_buffer[1] = 0x3A; tx_buffer[2] = 0x01;
                tx_buffer[3] = (adc_val >> 8) & 0xFF; tx_buffer[4] = adc_val & 0xFF;
                uint16_t sum = 0;
                for (uint8_t i = 0; i < 5; i++) sum += tx_buffer[i];
                tx_buffer[5] = (uint8_t)(sum >> 8); tx_buffer[6] = (uint8_t)(sum & 0xFF);
                tx_len = 7;
            } else { tx_buffer[0] = 0x3A; tx_buffer[1] = 0x00; tx_len = 2; }
        } else { tx_len = 0; }
    }
    /* --- Команда 0x3B: Запись по I2C (BitBang с fallback) --- */
    else if (cmd == 0x3B) {
        if (packet_len >= 4) {
            uint8_t device_addr = rx_buffer[2];
            uint8_t data_len = packet_len - 4;
            uint8_t *data_ptr = (uint8_t *) &rx_buffer[3];
            bool success = twi_bitbang_write_with_fallback(device_addr, data_ptr, data_len);
            tx_buffer[0] = 0x3B; tx_buffer[1] = success ? 0x01 : 0x00; tx_len = 2;
        } else { tx_len = 0; }
    }
    /* --- Команда 0x3C: Чтение по I2C (BitBang с fallback) --- */
    else if (cmd == 0x3C) {
        if (packet_len == 6) {
            uint8_t sensor_addr = rx_buffer[2];
            if (sensor_addr >= TMP75_ADDR_0 && sensor_addr <= TMP75_ADDR_3) {
                uint8_t temp_int, temp_frac;
                if (tmp75_read_with_fallback(sensor_addr, &temp_int, &temp_frac)) {
                    tx_buffer[0] = 0x07; tx_buffer[1] = 0x3C; tx_buffer[2] = 0x01;
                    tx_buffer[3] = temp_int; tx_buffer[4] = temp_frac; tx_buffer[5] = 0x00;
                    uint8_t resp_checksum = 0;
                    for (uint8_t i = 0; i < 6; i++) resp_checksum += tx_buffer[i];
                    tx_buffer[6] = resp_checksum; tx_len = 7;
                } else {
                    tx_buffer[0] = 0x07; tx_buffer[1] = 0x3C; tx_buffer[2] = 0x00;
                    tx_buffer[3] = 0x00; tx_buffer[4] = 0x00; tx_buffer[5] = 0x00;
                    uint8_t resp_checksum = 0;
                    for (uint8_t i = 0; i < 6; i++) resp_checksum += tx_buffer[i];
                    tx_buffer[6] = resp_checksum; tx_len = 7;
                }
            } else { tx_len = 0; }
        } else { tx_len = 0; }
    }
    /* --- Неизвестная команда --- */
    else { tx_len = 0; }
    
    tx_idx = 0;
}

/**
 * Главная функция.
 */
void main(void) {
    OSCCON = 0x78; /* 32 MHz */
    
    // --- ЧТЕНИЕ АДРЕСА С НОЖЕК RC3, RC4, RC5 ---
    
    // 1. Убеждаемся, что пины настроены как цифровые входы
    ANSELCbits.ANSC3 = 0; // Цифровой режим
    ANSELCbits.ANSC4 = 0;
    ANSELCbits.ANSC5 = 0;
    
    TRISCbits.TRISC3 = 1; // Вход
    TRISCbits.TRISC4 = 1; // Вход
    TRISCbits.TRISC5 = 1; // Вход
    
    // Небольшая задержка для стабилизации уровня на пинах после включения питания
    __delay_us(10);
    
    // 2. Читаем состояние пинов. 
    // Если на ножке есть напряжение (3.3V), бит будет 1. Если нет (0V/GND), бит будет 0.
    uint8_t chain_id = 0;
    
    if (PORTCbits.RC3) chain_id |= 0x01; // A0 (LSB)
    if (PORTCbits.RC4) chain_id |= 0x02; // A1
    if (PORTCbits.RC5) chain_id |= 0x04; // A2 (MSB)
    
    // 3. Вычисляем итоговый адрес
    // Примеры:
    // Все 0 (GND/Нет напряжения) -> 0x20 + 0 = 0x20
    // Только RC3 (3.3V)        -> 0x20 + 1 = 0x21
    // ...
    // Все 1 (3.3V)             -> 0x20 + 7 = 0x27
    
    uint8_t my_address = I2C_BASE_ADDRESS_MIN + chain_id;
    
    // --- ИНИЦИАЛИЗАЦИЯ ---
    
    I2C_Slave_Init(my_address);

    /* Настройка пина управления ключами питания чипов */
    POWER_KEY_ANSEL = 0;
    POWER_KEY_TRIS = 0;
    POWER_KEY_LAT = 0;

    while (1) {
        CLRWDT(); 
        
        // Обработка принятого пакета
        if (packet_ready) {
            INTCONbits.GIE = 0; // Блокируем прерывания на время обработки
            process_packet();
            packet_ready = false;
            INTCONbits.GIE = 1;
        }
        
        // Обработка сброса (команда 0x07) - ЗАМЕНЕНО НА SOFT RESET
        if (reset_pending) {
            if (tx_idx >= tx_len) {
                // Вместо RESET() делаем программный сброс состояния
                rx_state = RX_STATE_IDLE;
                rx_idx = 0;
                packet_len = 0;
                tx_len = 0;
                tx_idx = 0;
                packet_ready = false;
                
                // Очищаем буферы
                for (uint8_t i = 0; i < MAX_BUFFER_SIZE; i++) {
                    rx_buffer[i] = 0;
                    tx_buffer[i] = 0;
                }
                
                // Сбрасываем флаг
                reset_pending = false;
            }
        }
    }
}
