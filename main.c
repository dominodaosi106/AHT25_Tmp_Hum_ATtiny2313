/*
【ポート対応】 ATtiny2313A x:存在しない -:空き
bit       7     6     5     4     3     2     1     0
portA     x     x     x     x     x     x     SCL   SDA     I2C (AHT25)
portB     SegD  SegC  SegE  SegF  SegG  SegB  SegA  SegDP   7セグメントLED制御
portD     -     SW    Dig6  Dig5  Dig4  Dig3  Dig2  Dig1    7セグメント桁選択 (Dig6:スイッチ入力)

【ATtiny2313Aピンマッピング】
  (RESET) PA2 |1      20| VCC
   (Dig1) PD0 |2      19| PB7 (SegA)
   (Dig2) PD1 |3      18| PB6 (SegB)
   (SCL)  PA1 |4      17| PB5 (SegG)
   (SDA)  PA0 |5      16| PB4 (SegF)
   (Dig3) PD2 |6      15| PB3 (SegE)
   (Dig4) PD3 |7      14| PB2 (SegC)
   (Dig5) PD4 |8      13| PB1 (SegD)
   (Dig6) PD5 |9      12| PB0 (SegDP)
   GND        |10     11| PD6 (SW)
※SW: スイッチ入力（外部プルアップ4.7kΩ＋内部プルアップ）

【ビルド環境】
Atmel Studio 7 (Version: 7.0.129)
ATtiny2313A 内蔵1MHzクロック
プログラムメモリ: 約1894バイト（2048バイトの92.4%）
データメモリ: 約51バイト（128バイトの39.8%）
最適化: -Os適用
*/


#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

uint8_t seg[6], mx;
uint32_t temp, humd;
uint8_t display_mode = 0;
int16_t max_tem = -500, min_tem = 1500;
uint16_t max_hum = 0, min_hum = 1000;
uint8_t sensor_error = 1;
uint8_t dash_pos = 5, dash_dir = 0;
uint16_t timer_count = 0, anim_count = 0;

#define SDA PA0
#define SCL PA1
#define AHT25_ADDR 0x38

void delay_ms(uint16_t t) { while (t--) _delay_ms(1); }

uint8_t mask(uint8_t num) {
    static const uint8_t masks[] = {
				// ABGF ECDd
        0x21,	// 0010 0001  0: A, B, C, D, E, F点灯 (G, d消灯)
        0xBB,	// 1011 1011  1: B, C点灯 (A, G, F, E, D, d消灯)
        0x15,	// 0001 0101  2: A, B, D, E, G点灯 (F, C, d消灯)
        0x19,	// 0001 1001  3: A, B, C, D, G点灯 (F, E, d消灯)
        0x8B,	// 1000 1011  4: B, C, F, G点灯 (A, E, D, d消灯)
        0x49,	// 0100 1001  5: A, C, D, F, G点灯 (B, E, d消灯)
        0x41,	// 0100 0001  6: A, C, D, E, F, G点灯 (B, d消灯)
        0x2B,	// 0010 1011  7: A, B, C点灯 (G, F, E, D, d消灯)
        0x01,	// 0000 0001  8: A, B, C, D, E, F, G点灯 (d消灯)
        0x09,	// 0000 1001  9: A, B, C, D, F, G点灯 (E, d消灯)
        0xDF	// 1101 1111  -: G点灯 (A, B, F, E, C, D, d消灯)
    };
    return num <= 10 ? masks[num] : 0xFF;
}

ISR(TIMER0_COMPA_vect) {
    PORTD &= 0xC0;
    PORTB = seg[mx];
    switch (mx) {
        case 0: PORTD = 0x01; break;
        case 1: PORTD = 0x02; PORTB ^= 1; break;
        case 2: PORTD = 0x04; break;
        case 3: PORTD = 0x08; break;
        case 4: PORTD = 0x10; PORTB ^= 1; break;
        case 5: PORTD = 0x20; break;
    }
    mx = (mx + 1) % 6;
}

void i2c_init(void) { DDRA |= 1 << SCL; PORTA |= (1 << SDA) | (1 << SCL); }

uint8_t i2c_send(uint8_t data) {
    DDRA |= 1 << SDA;
    for (uint8_t i = 8; i; i--) {
        PORTA &= ~(1 << SCL);
        PORTA = (data & 0x80) ? PORTA | (1 << SDA) : PORTA & ~(1 << SDA);
        PORTA |= 1 << SCL;
        data <<= 1;
    }
    PORTA &= ~(1 << SCL); DDRA &= ~(1 << SDA); PORTA |= 1 << SDA;
    PORTA |= 1 << SCL; uint8_t ack = !(PINA & (1 << SDA));
    PORTA &= ~(1 << SCL); return ack;
}

uint8_t i2c_recv(uint8_t ack) {
    uint8_t data = 0; DDRA &= ~(1 << SDA); PORTA |= 1 << SDA;
    for (uint8_t i = 8; i; i--) {
        data <<= 1; PORTA &= ~(1 << SCL); PORTA |= 1 << SCL;
        if (PINA & (1 << SDA)) data |= 1;
    }
    PORTA &= ~(1 << SCL); DDRA |= 1 << SDA;
    PORTA = ack ? PORTA & ~(1 << SDA) : PORTA | (1 << SDA);
    PORTA |= 1 << SCL; PORTA &= ~(1 << SCL); DDRA &= ~(1 << SDA); PORTA |= 1 << SDA;
    return data;
}

void AHT25_init(void) {
    delay_ms(100);
    DDRA |= 1 << SDA; PORTA |= (1 << SDA) | (1 << SCL); PORTA &= ~(1 << SDA); PORTA &= ~(1 << SCL);
    i2c_send(AHT25_ADDR << 1); i2c_send(0x71); i2c_recv(1);
    DDRA |= 1 << SDA; PORTA &= ~(1 << SCL); PORTA &= ~(1 << SDA); PORTA |= 1 << SCL; PORTA |= 1 << SDA;
}

uint8_t AHT25_read(void) {
    uint8_t buf[6];
    DDRA |= 1 << SDA; PORTA |= (1 << SDA) | (1 << SCL); PORTA &= ~(1 << SDA); PORTA &= ~(1 << SCL);
    if (!i2c_send(AHT25_ADDR << 1)) goto stop;
    i2c_send(0xAC); i2c_send(0x33); i2c_send(0x00);
    DDRA |= 1 << SDA; PORTA &= ~(1 << SCL); PORTA &= ~(1 << SDA); PORTA |= 1 << SCL; PORTA |= 1 << SDA;
    delay_ms(80);
    DDRA |= 1 << SDA; PORTA |= (1 << SDA) | (1 << SCL); PORTA &= ~(1 << SDA); PORTA &= ~(1 << SCL);
    if (!i2c_send((AHT25_ADDR << 1) | 1)) goto stop;
    for (uint8_t i = 0; i < 5; i++) buf[i] = i2c_recv(1);
    buf[5] = i2c_recv(0);
    DDRA |= 1 << SDA; PORTA &= ~(1 << SCL); PORTA &= ~(1 << SDA); PORTA |= 1 << SCL; PORTA |= 1 << SDA;
    if (buf[0] & 0x80) return 0;
    humd = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    temp = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    return (humd || temp) ? 1 : 0;
stop:
    DDRA |= 1 << SDA; PORTA &= ~(1 << SCL); PORTA &= ~(1 << SDA); PORTA |= 1 << SCL; PORTA |= 1 << SDA;
    return 0;
}

void reset_display(void) {
    for (uint8_t i = 0; i < 6; i++) seg[i] = mask(8);
    delay_ms(1000);
}

void display_values(int16_t tem, uint16_t hum) {
    if (tem > 999 || tem < -99) {
        seg[5] = seg[4] = seg[3] = 0xFF;
    } else if (tem < 0) {
        seg[5] = mask(10); tem = -tem;
        seg[4] = mask(tem / 10); seg[3] = mask(tem % 10);
    } else {
        seg[5] = mask(tem / 100 ?: 99);
        seg[4] = mask((tem / 10) % 10); seg[3] = mask(tem % 10);
    }
    if (hum > 999) {
        seg[2] = seg[1] = seg[0] = 0xFF;
    } else {
        seg[2] = mask(hum / 100 ?: 99);
        seg[1] = mask((hum / 10) % 10); seg[0] = mask(hum % 10);
    }
}

int main(void) {
    int16_t tem = 0; uint16_t hum = 0;
    uint8_t btn_pressed = 0, seq_step = 0;
    uint16_t press_count = 0, disp_timer = 0;

    DDRB = 0xFF; DDRD = 0xBF; PORTD = 1 << PD6; // PD6にプルアップ
    TCCR0A = 0x02; TCCR0B = 0x02; OCR0A = 125; TIMSK = 1 << OCIE0A;
    i2c_init(); delay_ms(100); AHT25_init(); sei();
    for (uint8_t i = 0; i < 6; i++) seg[i] = 0xFF;
    seg[5] = 0x45; seg[4] = seg[3] = 0xD7;

    while (1) {
        // ボタン処理
        if (!(PIND & (1 << PD6))) { // ボタン押下（アクティブロー）
            press_count++;
            _delay_ms(5); // 5ms待機
            if (!btn_pressed && press_count >= 10) { // デバウンス（50ms）
                btn_pressed = 1; seq_step = 0; disp_timer = 0; display_mode = 1;
            }
            if (press_count >= 200) { // 長押し（4秒）
                max_tem = -500; min_tem = 1500; max_hum = 0; min_hum = 1000;
                reset_display(); seq_step = 2; display_mode = 0;
                press_count = 0; btn_pressed = 0;
                while (!(PIND & (1 << PD6))) _delay_ms(5); // ボタン離すまで待つ
            }
        } else {
            if (btn_pressed && press_count < 200) { // 短押し検出
                btn_pressed = 0; press_count = 0;
            } else if (press_count >= 200) { // 長押し完了後
                btn_pressed = 0; press_count = 0;
            }
        }

        if (++timer_count >= 25) {
            if (AHT25_read()) {
                sensor_error = 0;
                tem = ((temp * 2000L) >> 20) - 500;
                hum = (humd * 1000L) >> 20;
                if (tem > max_tem) max_tem = tem;
                if (tem < min_tem) min_tem = tem;
                if (hum > max_hum) max_hum = hum;
                if (hum < min_hum) min_hum = hum;
            } else sensor_error = 1;
            timer_count = 0;
        }

        if (seq_step < 2) {
            if (++disp_timer >= 100) { // 1秒表示
                seq_step++;
                disp_timer = 0;
                display_mode = seq_step == 1 ? 2 : 0;
            }
        }

        if (sensor_error) {
            if (++anim_count >= 50) {
                for (uint8_t i = 0; i < 6; i++) seg[i] = 0xFF;
                seg[dash_pos] = mask(10);
                dash_pos += dash_dir ? 1 : -1;
                if (dash_pos == 0 || dash_pos == 5) dash_dir ^= 1;
                anim_count = 0;
            }
        } else {
            display_values(display_mode == 0 ? tem : display_mode == 1 ? max_tem : min_tem,
                           display_mode == 0 ? hum : display_mode == 1 ? max_hum : min_hum);
            anim_count = 0;
        }
        _delay_ms(1);
    }
}