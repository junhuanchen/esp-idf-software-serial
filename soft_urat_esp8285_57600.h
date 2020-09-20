/*
Github: junhuanchen
Copyright (c) 2018 Juwan
Licensed under the MIT license:
http://www.opensource.org/licenses/mit-license.php
*/

#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/portmacro.h>
#include "esp_freertos_hooks.h"

#include <esp_clk.h>
#include <driver/gpio.h>
#include "driver/soc.h"
#include "rom/ets_sys.h"

#define SW_EOF -1 

typedef struct sw_serial
{
    uint32_t rxPin, txPin;
    uint32_t buffSize, bitTime, rx_start_time, rx_end_time;
    bool invert, overflow;
    volatile uint32_t inPos, outPos;
    uint8_t *buffer;
} SwSerial;

SwSerial *sw_new(uint32_t Tx, uint32_t Rx, bool Inverse, int buffSize)
{
    SwSerial *tmp = (SwSerial *)malloc(sizeof(SwSerial));

    if (NULL != tmp)
    {
        tmp->invert = Inverse;
        tmp->overflow = false;
        tmp->inPos = tmp->outPos = 0;
        tmp->buffSize = buffSize;
        tmp->buffer = (uint8_t *)malloc(buffSize);
        if (NULL != tmp->buffer)
        {
            gpio_config_t io_conf;
            io_conf.intr_type = Inverse ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;
            io_conf.pin_bit_mask = 1ULL << Rx;
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = 1;
            io_conf.pull_down_en = 1;
            gpio_config(&io_conf);

            // disable interrupt
            io_conf.intr_type = GPIO_INTR_DISABLE;
            // set as output mode
            io_conf.mode = GPIO_MODE_OUTPUT;
            // bit mask of the pins that you want to set
            io_conf.pin_bit_mask = (1ULL << Tx);
            // disable pull-down mode
            io_conf.pull_down_en = 1;
            // disable pull-up mode
            io_conf.pull_up_en = 1;
            // configure GPIO with the given settings
            ESP_ERROR_CHECK(gpio_config(&io_conf));
            ESP_ERROR_CHECK(gpio_set_level(Tx, Inverse));

            tmp->txPin = Tx;
            tmp->rxPin = Rx;
                    
            // For the TTL level of positive logic, the starting bit is the low level of one bit time.
            // gpio_set_level(Tx, !Inverse); 
            // Too short leads to sticky bags
            // One byte of time 9600 104us * 10 115200 18us
            vTaskDelay(2 / portTICK_RATE_MS);

            return tmp;
        }
        free(tmp), tmp = NULL;
    }

    return tmp;
}

void sw_del(SwSerial *self)
{
    if (NULL != self->buffer)
    {
        free(self->buffer);
    }
    
    free(self);
}

// uint32_t getCycleCount()
// {
//     uint32_t ccount;
//     __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
//     return ccount;
// }


#define WaitBitTime(wait) \
    for (uint32_t start = (soc_get_ccount()); (soc_get_ccount()) - start < (wait);)

// 0.1 us == soc_get_ccount() / g_esp_ticks_per_us / 10

// #define WaitBitTime(wait) ets_delay_us(8)

static inline bool IRAM_ATTR wait_bit_state(uint8_t pin, uint8_t state, uint8_t limit)
{
    for (uint i = 0; i != limit; i++)
    {
        // ets_delay_us(1);
        WaitBitTime(limit);
        if (state == gpio_get_level(pin))
        {
            return true;
        }
    }
    // ets_delay_us(1);
    return false;
}

static inline uint8_t IRAM_ATTR check_bit_state(uint8_t pin, uint8_t limit)
{
    uint8_t flag[2] = { 0 };
    for (uint i = 0; i != limit; i++)
    {
        flag[gpio_get_level(pin)] += 1;
        ets_delay_us(1);
    }
    return flag[0] < flag[1];// flag[0] < flag[1] ? 1 : 0;
}

// The first byte will wrong, after normal
static void IRAM_ATTR sw_rx_handler(void *args)
{
    portENTER_CRITICAL();
    SwSerial *self = (SwSerial *)args;
    uint8_t rec = 0;
    
    // portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    // (self->invert) flag invert set Start 1 And Stop 0 invert
    // But myself not need, so need extra added by yourself

    // Wait Start Bit To Start
    // WaitBitTime(self->rx_start_time);
    // if (self->invert == gpio_get_level(self->rxPin))
    if (wait_bit_state(self->rxPin, self->invert, self->rx_start_time))
    {
        for (uint8_t i = 0; i != 8; i++)
        {
            rec >>= 1;
            uint32_t tmp = self->bitTime - (i * 35);
            WaitBitTime(tmp);
            if (gpio_get_level(self->rxPin))
            {
                rec |= 0x80;
            }
        }
        // Wait Start Bit To End
        // WaitBitTime(self->rx_end_time);
        // if (!self->invert == gpio_get_level(self->rxPin))
        if (wait_bit_state(self->rxPin, !self->invert, self->rx_end_time))
        {
            // Stop bit Allow Into RecvBuffer
            // Store the received value in the buffer unless we have an overflow
            int next = (self->inPos + 1) % self->buffSize;
            if (next != self->outPos)
            {
                self->buffer[self->inPos] = (self->invert) ? ~rec : rec;
                self->inPos = next;
            }
            else
            {
                self->overflow = true;
            }
        }
    }
    
    portEXIT_CRITICAL();
    // Must clear this bit in the interrupt register,
    // it gets set even when interrupts are disabled

    // Esp32 GPIO.status_w1tc interrupt auto recovery
}

esp_err_t sw_enableRx(SwSerial *self, bool State)
{
    esp_err_t error = ESP_OK;
    if (State)
    {
        gpio_set_intr_type(self->rxPin, (self->invert) ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE);
        gpio_install_isr_service(0);
        error = gpio_isr_handler_add(self->rxPin, sw_rx_handler, (void*)self);

    }
    else
    {
        error = gpio_isr_handler_remove(self->rxPin);
        gpio_uninstall_isr_service();
    }
    
    return error;
}

int sw_write(SwSerial *self, uint8_t byte)
{
    if (self->invert)
    {
        byte = ~byte;
    }
    
    // Disable interrupts in order to get a clean transmit
    // portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL();

    // create tx interrupts to start bit.
    gpio_set_level(self->txPin, 0);
    gpio_set_level(self->txPin, 1);
    // WaitBitTime(self->bitTime);
    ets_delay_us(8 * 2); // 115200 = 8 * 1 us

    for (uint8_t i = 0; i != 8; i++)
    {
        gpio_set_level(self->txPin, (byte & 1) ? 1 : 0);
        ets_delay_us(8 * 2); // 115200 = 8 * 1 us

        byte >>= 1;
    }

    // Stop bit
    gpio_set_level(self->txPin, 0);
    ets_delay_us(8 * 2); // 115200 = 8 * 1 us

    // re-enable interrupts
    portEXIT_CRITICAL();
    
    return 1;
}

int sw_read(SwSerial *self)
{
    if (self->inPos != self->outPos)
    {
        uint8_t ch = self->buffer[self->outPos];
        self->outPos = (self->outPos + 1) % self->buffSize;
        return ch;
    }
    return -1;
}

// suggest max datalen <= 256 and baudRate <= 115200
esp_err_t sw_open(SwSerial *self, uint32_t baudRate)
{
    // The oscilloscope told me
    self->bitTime = (esp_clk_cpu_freq() / (baudRate));

    // Rx bit Timing Settings
    switch (baudRate)
    {
        case 115200:
            self->rx_start_time = (self->bitTime / 256);
            self->rx_end_time = (self->bitTime / 256);
            break;
        
        case 9600:
            self->rx_start_time = (self->bitTime / 9);
            self->rx_end_time = (self->bitTime * 8 / 9);
            break;
        
        default: // tested 57600 len 256
            self->rx_start_time = (self->bitTime / 9);
            self->rx_end_time = (self->bitTime / 9);
            break;
    }
    
    // printf("sw_open %u %d\n", self->rx_start_time, self->rx_end_time);

    sw_write(self, 0x00); // Initialization uart link

    return sw_enableRx(self, true);
}

esp_err_t sw_stop(SwSerial *self)
{
    return sw_enableRx(self, false);
}

int sw_any(SwSerial *self)
{
    int avail = self->inPos - self->outPos;
    return (avail < 0) ? avail + self->buffSize : avail;
}

void sw_flush(SwSerial *self)
{
    self->inPos = self->outPos = 0;
    self->overflow = false;
}

bool sw_overflow(SwSerial *self)
{
    return self->overflow;
}

int sw_peek(SwSerial *self)
{
    if (self->inPos != self->outPos)
    {
        return self->buffer[self->outPos];
    }
    return -1;
}