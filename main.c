/*
 * Project: Bitmain Antminer S19 Series PIC16F1704 Firmware Patch-Fix
 * Author: Anton Vinogradov @vinantole support@minerflash.ru
 * Year: 2026
 * 
 * Description:
 * Полностью переписанный патч-фикс оригинальной прошивки PIC-контроллера
 * для хеш-плат Bitmain Antminer S19 серий.
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 32000000UL

#pragma config FOSC = INTOSC, WDTE = OFF, PWRTE = OFF, MCLRE = ON, CP = OFF, BOREN = ON
#pragma config CLKOUTEN = OFF, IESO = ON, FCMEN = ON, WRT = OFF, PPS1WAY = ON, PLLEN = ON
#pragma config STVREN = ON, BORV = LO, LPBOR = OFF, LVP = ON

#define I2C_BASE_ADDRESS    0x20
#define MAX_BUFFER_SIZE     16
#define FW_VERSION   0xAB

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

void process_packet(void);
void handle_i2c_event(void);
uint16_t adc_read_channel(uint8_t channel);
bool tmp75_read_with_fallback(uint8_t primary_addr, uint8_t *temp_int, uint8_t *temp_frac);
bool twi_bitbang_write_with_fallback(uint8_t target_addr, uint8_t *data, uint8_t len);
void twi_bitbang_init_pins(void);
void twi_bitbang_start(void);
void twi_bitbang_stop(void);
bool twi_bitbang_write_byte(uint8_t data);
uint8_t twi_bitbang_read_byte(bool ack);
void I2C_Slave_Init(void);

/**
Проверка дополнительных байтов пакета на равенство нулю.
Возвращает true если все байты после команды (до контрольной суммы) равны 0x00.
Используется для валидации пакетов команд 0x01-0x09, 0x17, 0x3A.
*/
bool check_extra_bytes_zero(void) {
    for (uint8_t i = 2; i < packet_len - 1; i++) {
        if (rx_buffer[i] != 0x00) {
            return false;
        }
    }
    return true;
}

/**
 * Инициализация пинов BitBang I2C для общения с датчиками температуры.
 */
void twi_bitbang_init_pins(void) {
    ANSELAbits.ANSA4 = 0;
    TWI_SDA_TRIS = 1;
    TWI_SCL_TRIS = 1;
    TWI_SDA_LAT = 0;
    TWI_SCL_LAT = 0;
}

/**
 * Генерация условия START на шине I2C.
 */
void twi_bitbang_start(void) {
    TWI_SDA_TRIS = 1;
    TWI_SCL_TRIS = 1;
    TWI_DELAY();
    TWI_SDA_TRIS = 0;
    TWI_DELAY();
    TWI_SCL_TRIS = 0;
    TWI_DELAY();
}

/**
 * Генерация условия STOP на шине I2C.
 */
void twi_bitbang_stop(void) {
    TWI_SDA_TRIS = 0;
    TWI_DELAY();
    TWI_SCL_TRIS = 1;
    TWI_DELAY();
    TWI_SDA_TRIS = 1;
    TWI_DELAY();
}

/**
 * Передача одного байта по BitBang I2C.
 * Возвращает true при получении ACK от ведомого устройства.
 */
bool twi_bitbang_write_byte(uint8_t data) {
    uint8_t i;
    bool ack;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) TWI_SDA_TRIS = 1; else TWI_SDA_TRIS = 0;
        TWI_DELAY();
        TWI_SCL_TRIS = 1;
        TWI_DELAY();
        TWI_SCL_TRIS = 0;
        data <<= 1;
    }
    TWI_SDA_TRIS = 1;
    TWI_DELAY();
    TWI_SCL_TRIS = 1;
    TWI_DELAY();
    ack = !TWI_SDA_PORT;
    TWI_SCL_TRIS = 0;
    TWI_DELAY();
    return ack;
}

/**
 * Чтение одного байта с шины BitBang I2C.
 * @param ack true - отправить ACK, false - отправить NACK
 */
uint8_t twi_bitbang_read_byte(bool ack) {
    uint8_t i;
    uint8_t data = 0;
    TWI_SDA_TRIS = 1;
    for (i = 0; i < 8; i++) {
        data <<= 1;
        TWI_SCL_TRIS = 1;
        TWI_DELAY();
        if (TWI_SDA_PORT) data |= 0x01;
        TWI_SCL_TRIS = 0;
        TWI_DELAY();
    }
    if (ack) TWI_SDA_TRIS = 0; else TWI_SDA_TRIS = 1;
    TWI_DELAY();
    TWI_SCL_TRIS = 1;
    TWI_DELAY();
    TWI_SCL_TRIS = 0;
    TWI_DELAY();
    TWI_SDA_TRIS = 1;
    return data;
}

/**
 * Чтение температуры с датчика TMP75 с механизмом fallback.
 * Если основной адрес не отвечает, пытается опросить адрес 0x48.
 * Патч фикс для опроса неисправных датчиков температуры
 * Если на хэш плате нет не одного исправного датчика, то вернёт ошику в ответе: 07 3C 00 00 00 00 43 
 */
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

/**
 * Запись данных по BitBang I2C с механизмом fallback.
 * Пробует несколько адресов из списка, если основной не отвечает.
 * Патч фикс для опроса неисправных датчиков температуры
 * Если на хэш плате нет не одного исправного датчика, то вернёт ошику в ответе: 3A 00  
 */
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

/**
 * Чтение значения ADC с указанного канала.
 * Канал 0x06 соответствует ножке RC2 (AN6) на PIC16F1704.
 */
uint16_t adc_read_channel(uint8_t channel) {
    ADCON1 = 0xD0; /* Fosc/64, Right justified, Vref=VDD/VSS */
    ADCON0 = (uint8_t)((channel << 2) | 0x01);
    __delay_us(10); /* Acquisition time */
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE);
    return ((uint16_t)ADRESH << 8) | ADRESL;
}

/**
 * Инициализация аппаратного I2C Slave.
 * Адрес устройства: базовый 0x20 (без смещения, Flash не используется).
 */
void I2C_Slave_Init(void) {
    uint8_t i2c_address = I2C_BASE_ADDRESS;

    ANSELC = 0x00;
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC1 = 1;
    SSPCLKPPS = 0x10; /* RC0 -> SCL */
    SSPDATPPS = 0x11; /* RC1 -> SDA */
    RC0PPS = 0x10;    /* SCL -> RC0 */
    RC1PPS = 0x11;    /* SDA -> RC1 */
    SSPADD = (uint8_t)(i2c_address << 1);
    SSPCON1 = 0x36;   /* I2C Slave, 7-bit address */
    SSPCON2 = 0x01;
    SSPCON3 = 0x18;   /* BOEN + SDAHT */
    PIR1bits.SSP1IF = 0;
    PIE1bits.SSP1IE = 0;
}

/**
 * Главная функция. Инициализация периферии и главный цикл обработки I2C.
 */
void main(void) {
    OSCCON = 0x78; /* 16 MHz HFINTOSC, PLL включен через конфиг -> 32 MHz */
    I2C_Slave_Init();

    /* Настройка пина управления ключами питания чипов */
    POWER_KEY_ANSEL = 0;
    POWER_KEY_TRIS = 0;
    POWER_KEY_LAT = 0;

    while (1) {
        if (PIR1bits.SSP1IF) handle_i2c_event();
        if (reset_pending && tx_idx >= tx_len && !SSPSTATbits.S && SSPSTATbits.P) {
            for (volatile uint16_t i = 0; i < 5000; i++);
            RESET();
        }
    }
}

/**
 * Обработчик событий MSSP (I2C Slave).
 * Реализует конечный автомат приёма пакетов: PREAMBLE(0x55) -> LENGTH -> DATA.
 */
void handle_i2c_event(void) {
    uint8_t dummy;
    if (SSPCON1bits.WCOL) {
        SSPCON1bits.WCOL = 0;
        dummy = SSPBUF;
        rx_state = RX_STATE_IDLE;
    } else if (SSPSTATbits.R_nW) {
        if (SSPSTATbits.BF && !SSPSTATbits.D_nA) {
            dummy = SSPBUF;
            if (tx_idx < tx_len) SSPBUF = tx_buffer[tx_idx]; else SSPBUF = 0xFF;
        } else {
            tx_idx++;
        }
    } else if (SSPSTATbits.BF) {
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
                        process_packet();
                        rx_state = RX_STATE_IDLE;
                    }
                    break;
            }
        }
    }
    SSPCON1bits.CKP = 1;
    PIR1bits.SSP1IF = 0;
}

/**
Обработка принятого пакета.
Проверяет контрольную сумму и формирует ответ в зависимости от команды.
Все команды допускают дополнительный байт 0x00 в конце пакета.
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
    
    /* --- Команды 0x01..0x05,0x08,0x09: Flash-заглушки (Flash удалён в этой версии) --- */

    /* --- Команда 0x01 SET_PIC_FLASH_POINTER, ACK --- */
    /* --- Команда 0x02 SEND_DATA_TO_IIC, ACK --- */
    if (cmd >= 0x01 && cmd <= 0x02) {
        tx_buffer[0] = cmd;
        tx_buffer[1] = 0x00;
        tx_len = 2;
    }     
    /* --- Команда 0x03 READ_DATA_FROM_IIC, ACK --- */
    /* --- Команда 0x04 ERASE_IIC_FLASH, ACK --- */
    /* --- Команда 0x05 WRITE_DATA_INTO_PIC, ACK --- */     
    else if (cmd >= 0x03 && cmd <= 0x05) {
        if (!check_extra_bytes_zero()) {
            tx_buffer[0] = 0xFF;
            tx_buffer[1] = 0xFF;
            tx_len = 2;
        } else {
            tx_buffer[0] = cmd;
            tx_buffer[1] = 0x00;
            tx_len = 2;
        }
    }
    /* --- Команда 0x08 GET_PIC_FLASH_POINTER, ACK --- */ 
    else if (cmd == 0x08) {
        if (!check_extra_bytes_zero()) {
            tx_buffer[0] = 0xFF;
            tx_buffer[1] = 0xFF;
            tx_len = 2;
        } else {
            tx_buffer[0] = 0x08;
            tx_buffer[1] = 0x00;
            tx_len = 2;
        }
    } 
    /* --- Команда 0x09 ERASE_PIC_APP_PROGRAM, ACK --- */ 
    else if (cmd == 0x09) {
        if (!check_extra_bytes_zero()) {
            tx_buffer[0] = 0xFF;
            tx_buffer[1] = 0xFF;
            tx_len = 2;
        } else {
            tx_buffer[0] = 0x09;
            tx_buffer[1] = 0x00;
            tx_len = 2;
        }
    } 
    /* --- Команда 0x06: JUMP_FROM_LOADER_TO_APP, ACK --- */
    else if (cmd == 0x06) {
        if (!check_extra_bytes_zero()) {
            tx_buffer[0] = 0xFF;
            tx_buffer[1] = 0xFF;
            tx_len = 2;
        } else {
            tx_buffer[0] = 0x06;
            tx_buffer[1] = 0x01;
            tx_len = 2;
        }
    }
    /* --- Команда 0x07: RESET_PIC, ACK + сброс --- */
    else if (cmd == 0x07) {
        if (!check_extra_bytes_zero()) {
            tx_buffer[0] = 0xFF;
            tx_buffer[1] = 0xFF;
            tx_len = 2;
        } else {
            tx_buffer[0] = 0x07;
            tx_buffer[1] = 0x01;
            tx_len = 2;
            reset_pending = true;
        }
    }
    /* --- Команда 0x15: ENABLE_VOLTAGE --- */
    else if (cmd == 0x15) {
        if (packet_len >= 3) {
            uint8_t state = rx_buffer[2];
            if (state == 0x01) {
                POWER_KEY_LAT = 1; /* Включить питание чипов */
            } else {
                POWER_KEY_LAT = 0; /* Выключить питание чипов */
            }
            tx_buffer[0] = 0x15;
            tx_buffer[1] = 0x01;
            tx_len = 2;
        } else {
            tx_len = 0;
        }
    }
    /* --- Команда 0x16: SEND_HEART_BEAT --- */
    else if (cmd == 0x16) {
        uint8_t status_byte = 0x00;
        tx_buffer[0] = 0x06;
        tx_buffer[1] = 0x16;
        tx_buffer[2] = 0x01;
        tx_buffer[3] = status_byte;
        uint16_t resp_sum = 0;
        for (uint8_t i = 0; i < 4; i++) resp_sum += tx_buffer[i];
        tx_buffer[4] = (uint8_t)(resp_sum >> 8);
        tx_buffer[5] = (uint8_t)(resp_sum & 0xFF);
        tx_len = 6;
    }
    /* --- Команда 0x17: GET_PIC_SOFTWARE_VERSION (0xAB = Антон Виноградов) --- */
    else if (cmd == 0x17) {
        if (!check_extra_bytes_zero()) {
            tx_buffer[0] = 0xFF;
            tx_buffer[1] = 0xFF;
            tx_len = 2;
        } else {
            tx_buffer[0] = 0x05;
            tx_buffer[1] = 0x17;
            tx_buffer[2] = FW_VERSION;
            tx_buffer[3] = 0x00;
            tx_buffer[4] = 0xC7;
            tx_len = 5;
        }
    }
    /* --- Команда 0x3A: Чтение ADC --- */
    /* Канал 0x06 задаётся автоматически. Другие каналы -> NACK (0x3A 0x00) */
    else if (cmd == 0x3A) {
        if (!check_extra_bytes_zero()) {
            tx_buffer[0] = 0xFF;
            tx_buffer[1] = 0xFF;
            tx_len = 2;
        } else if (packet_len >= 3) {
            uint8_t channel = rx_buffer[2];
            if (channel == 0x00) {
                /* Корректный запрос: читаем фиксированный канал 0x06 (RC2/AN6) */
                uint16_t adc_val = adc_read_channel(0x06);
                tx_buffer[0] = 0x07;
                tx_buffer[1] = 0x3A;
                tx_buffer[2] = 0x01;
                tx_buffer[3] = (adc_val >> 8) & 0xFF;
                tx_buffer[4] = adc_val & 0xFF;
                uint16_t sum = 0;
                for (uint8_t i = 0; i < 5; i++) sum += tx_buffer[i];
                tx_buffer[5] = (uint8_t)(sum >> 8);
                tx_buffer[6] = (uint8_t)(sum & 0xFF);
                tx_len = 7;
            } else {
                /* Неправильный канал: отвечаем NACK */
                tx_buffer[0] = 0x3A;
                tx_buffer[1] = 0x00;
                tx_len = 2;
            }
        } else {
            tx_len = 0;
        }
    }
    /* --- Команда 0x3B: Запись по I2C (BitBang с fallback) --- */
    else if (cmd == 0x3B) {
        if (packet_len >= 4) {
            uint8_t device_addr = rx_buffer[2];
            uint8_t data_len = packet_len - 4;
            uint8_t *data_ptr = (uint8_t *) &rx_buffer[3];
            bool success = twi_bitbang_write_with_fallback(device_addr, data_ptr, data_len);
            tx_buffer[0] = 0x3B;
            tx_buffer[1] = success ? 0x01 : 0x00;
            tx_len = 2;
        } else {
            tx_len = 0;
        }
    }
    /* --- Команда 0x3C: Чтение по I2C (BitBang с fallback) --- */
    else if (cmd == 0x3C) {
        if (packet_len == 6) {
            uint8_t sensor_addr = rx_buffer[2];
            if (sensor_addr >= TMP75_ADDR_0 && sensor_addr <= TMP75_ADDR_3) {
                uint8_t temp_int, temp_frac;
                if (tmp75_read_with_fallback(sensor_addr, &temp_int, &temp_frac)) {
                    tx_buffer[0] = 0x07;
                    tx_buffer[1] = 0x3C;
                    tx_buffer[2] = 0x01;
                    tx_buffer[3] = temp_int;
                    tx_buffer[4] = temp_frac;
                    tx_buffer[5] = 0x00;
                    uint8_t resp_checksum = 0;
                    for (uint8_t i = 0; i < 6; i++) resp_checksum += tx_buffer[i];
                    tx_buffer[6] = resp_checksum;
                    tx_len = 7;
                } else {
                    tx_buffer[0] = 0x07;
                    tx_buffer[1] = 0x3C;
                    tx_buffer[2] = 0x00;
                    tx_buffer[3] = 0x00;
                    tx_buffer[4] = 0x00;
                    tx_buffer[5] = 0x00;
                    uint8_t resp_checksum = 0;
                    for (uint8_t i = 0; i < 6; i++) resp_checksum += tx_buffer[i];
                    tx_buffer[6] = resp_checksum;
                    tx_len = 7;
                }
            } else {
                tx_len = 0;
            }
        } else {
            tx_len = 0;
        }
    }
    /* --- Неизвестная команда --- */
    else {
        tx_len = 0;
    }
    
    tx_idx = 0;
}
