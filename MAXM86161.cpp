#include "MAXM86161.h"

/*****************************************************************************/
// Constructor
/*****************************************************************************/
/*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*/
MAXM86161::MAXM86161(I2C &i2c):
_i2cbus(i2c)
{
    // Empty block
}

MAXM86161::~MAXM86161(void)
{
    // Empty block
}


// Initialize the sensor
// Sets the PPG sensor to starting condition, then puts it in SHDN mode ready to
// take data
int MAXM86161::init(void)
{
    int read_value;
    // Use function to do software reset
    _write_to_reg(REG_SYSTEM_CONTROL, 0x09);

    ThisThread::sleep_for(1ms);

    // Shut Down
    _write_to_reg(REG_SYSTEM_CONTROL, 0x0A);

    ThisThread::sleep_for(2ms);

    // Clear Interrupt 1 by reading
    _read_from_reg(REG_IRQ_STATUS1, &read_value);

    // Clear Interrupt 2 by reading
    _read_from_reg(REG_IRQ_STATUS2, &read_value);

    // Set integration time and ADC range with ALC and ADD
    _write_to_reg(REG_PPG_CONFIG1, 0x0F);

    // Set sample rate and averaging
    // cmd[0] = 0x12; cmd[1] = 0x50;  // 8Hz with no averaging
    // cmd[1] = 0x08; 50 Hz with no averaging
    _write_to_reg(REG_PPG_CONFIG2, 0x08);

    // Set LED settling, digital filter, burst rate, burst enable
    //  No Burst mode with default settling
    _write_to_reg(REG_PPG_CONFIG3, 0x40);

    // Set Photodiode bias to 0pF to 65pF
    _write_to_reg(REG_PD_BIAS, 0x40);

    // Set LED driver range to 124 mA
    _write_to_reg(REG_LED_RANGE1, 0x3F);

    // Set LED current
    _write_to_reg(REG_LED1_PA, 0x14);
    _write_to_reg(REG_LED2_PA, 0x14);
    _write_to_reg(REG_LED3_PA, 0x14);

    // Enable Low Power Mode
    _write_to_reg(REG_SYSTEM_CONTROL, 0xC);

    //********************************
    // FIFO enable

    // Set FIFO full to 15 empty spaces left
    _write_to_reg(REG_FIFO_CONFIG1, 0xF);

    // Enable FIFO rollover when full
    _write_to_reg(REG_FIFO_CONFIG2, 0x2);

    // Enable FIFO interrupt
    _write_to_reg(REG_IRQ_ENABLE1, 0x80);

    // Set LED exposure to timeslots
    _write_to_reg(REG_LED_SEQ1, 0x43); //LED2 to time slot 1 and LED3 to time slot 2
    _write_to_reg(REG_LED_SEQ2, 0x00);
    _write_to_reg(REG_LED_SEQ3, 0x00);

    // Shutdown at the end and wait for signal to start
    _write_to_reg(REG_SYSTEM_CONTROL, 0b00001110);

    // Read device ID, if it matches the value for MAXM86161, return 0, otherwise return 1.

    // TODO: Write a function to test if the sensor can be communicated with
    return 0;
}

int MAXM86161::start(void)
{
    
    // Shutdown at the end and wait for signal to start
    // TODO: Get value of register

    // Apply mask

    // Write to the register
    _write_to_reg(REG_SYSTEM_CONTROL, 0b00001100);
    return 0;
}

int MAXM86161::stop(void)
{
    // Shutdown at the end and wait for signal to start
    _write_to_reg(REG_SYSTEM_CONTROL, 0b00001110);
    return 0;
}

// Function to read from a registry
int MAXM86161::_read_from_reg(int address, int* data){
    char cmd[16];
    char rsp[256];
    int status;
    cmd[0] = address;
    status = _i2cbus.write(PPG_ADDR, cmd, 1, true);
    if(status !=0) {
        // serial_pc.printf("Failed to write register address %#X during read.\n", address);
        return status;
        }
    
    status = _i2cbus.read(PPG_ADDR, rsp, 1);
    if(status !=0){        
        // serial_pc.printf("Failed to read register %#X value during read.\n", address);
        return status;
        }

    data = rsp[0];
    // serial_pc.printf("\n\r %#X Status: %#X\n\r", address, rsp[0]);
    return status;
}

// Function to write to a registry
// TODO Check about mfio -> might not need it.
int MAXM86161::_write_to_reg(int address, int value) {
    char cmd[16];
    char rsp[256];
    int status;
    cmd[0] = address; cmd[1] = value;
    status = _i2cbus.write(PPG_ADDR, cmd, 2);
    if(status !=0) {
        // serial_pc.printf("write error %#x\n", address);
        }
    else {
    // mfio = 1; thread_sleep_for(2); mfio = 0; wait_us(300);
    // ppg_i2c.read(SH_ADDR, rsp, 1);
    // mfio = 1; thread_sleep_for(2); mfio = 0; wait_us(300);

    // wait_us(300);
    _i2cbus.read(PPG_ADDR, rsp, 1);
    // wait_us(300);
    // serial_pc.printf("\n\r %#X Status: %#X\n\r", address, rsp[0]);
    }
    return status;
}




/*
// Online C++ compiler to run C++ program online
#include <iostream>
#include <stdio.h>
#include <bitset>

int rsp = 0;

int count = 15;
int i = 0;
int main() {
    // Write C++ code here
    std::cout << "Hello world!\n\n";
    
    for ( i=0; i<count; i++ ){
        rsp = rsp + 1;
        std::cout << rsp << "\n";
    }
    
    char a = 255;
    std::bitset<8> x(a);
    std::cout << x << '\n';
    x &= ~(1<<3);
    // x = x << 1;
    std::cout << x << "\n";


    x |= (1 << 5) | (1 << 7); 
    std::cout << x << "\n";
    
    
    int c = 0;
    std::bitset<8> y(c);
    std::cout << y << '\n';
    
    y &= ~(1<<3);
    // x = x << 1;
    std::cout << y << "\n";


    y |= (1 << 5) | (1 << 7); 
    std::cout << y << "\n";
    
    return 0;
}

*/
