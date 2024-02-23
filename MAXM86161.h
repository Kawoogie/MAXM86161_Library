/**
 *  Code to get the raw data from a MAX86161
 *  
 *  This was written for use with Mbed OS 6.16.0 on a MAX32630FTHR
 * 
 * Example of use:
 * @code
 * #include "mbed.h"
 * #include "MAXM86161.h"
 * #include "USBSerial.h"  // For communication via the onboard USB port
 * 
 * 
 * // Start serial communication
 * USBSerial serial;
 * 
 * // Create the Sensor_Raw object
 * Sensor_Raw sensor_hub;
 * 
 * int main()
 * {   
 *     // Initialize the sensor hub with default values
 *     sensor_hub.init_sh_raw();
 *     
 *     // Set the data interrogation rate to 50 Hz.
 *     sensor_hub.set_sample_rate(50);
 *     
 *     // Set the drive current for all 3 LEDs 
 *     sensor_hub.set_led_current(0x12);
 *  
 *     // Variables to hold the ppg data
 *     int red_ppg, green_ppg, ir_ppg;
 * 
 *     while (true) {
 *       // Read the data at a rate of 50 Hz and output the information via serial
 *       sensor_hub.read_sh_fifo(&red_ppg, &green_ppg, &ir_ppg);
 *       serial.printf("%d,%d,%d,\n\r", red_ppg, green_ppg, ir_ppg);
 *       ThisThread::sleep_for(20ms);
    }
 * }
 * @endcode
 * 
*/
#include "mbed.h"
#include <stdint.h>


class MAXM86161 {
    public:
    /** @brief Constructor
     * @param i2c Pointer for bus for communication via I2C
    */
    MAXM86161(I2C &i2c);

    /** @brief Destructor */
    ~MAXM86161(void);

    int init(void);
    int start(void);
    int stop(void);
    // void read_fifo(int* red, int* green, int* ir);
    // void led_off();
    // void set_led_current(char brightness);
    // void set_led_red_current(char brightness);
    // void set_led_green_current(char brightness);
    // void set_led_ir_current(char brightness);
    // void set_sample_rate(int rate);
    // char read_package_temp();

    // Functions not yet working
    // char read_status();




    private:
    DigitalInOut mfio;
    DigitalInOut rst;
    InterruptIn irq_pin;
    I2C & _i2cbus;

    // void _set_led_sequence(char sequence);

    int _read_from_reg(int address, int* data);
    int _write_to_reg(int address, int* value);

};



// Define the I2C address of the PPG sensor
#define PPG_ADDR 0xC4


/*******************************************************************************
 ************************** Maxm86161 I2C Registers *******************************
 ******************************************************************************/

//Status group
#define REG_IRQ_STATUS1		0x00
#define REG_IRQ_STATUS2		0x01
#define REG_IRQ_ENABLE1		0x02
#define REG_IRQ_ENABLE2		0x03

//FIFO group
#define REG_FIFO_WRITE_PTR	  0x04
#define REG_FIFO_READ_PTR		  0x05
#define REG_OVF_COUNTER			  0x06
#define REG_FIFO_DATA_COUNTER	0x07
#define REG_FIFO_DATA			    0x08
#define REG_FIFO_CONFIG1		  0x09
#define REG_FIFO_CONFIG2		  0x0A

//System
#define REG_SYSTEM_CONTROL		0x0D

//PPG configuration
#define REG_PPG_SYNC_CONTROL	  0x10
#define REG_PPG_CONFIG1			    0x11
#define REG_PPG_CONFIG2			    0x12
#define REG_PPG_CONFIG3			    0x13
#define REG_PROX_INT_THRESHOLD	0x14
#define REG_PD_BIAS				      0x15

//PPG picket fence detect and replace
#define REG_PICKET_FENCE	0x16

//LEd sequence control
#define REG_LED_SEQ1		0x20
#define REG_LED_SEQ2		0x21
#define REG_LED_SEQ3		0x22

//LED pulse amplitude
#define REG_LED1_PA			  0x23
#define REG_LED2_PA			  0x24
#define REG_LED3_PA			  0x25
#define REG_LED_PILOT_PA	0x29
#define REG_LED_RANGE1		0x2A

//PPG1_HI_RES_DAC
#define REG_S1_HI_RES_DAC1	0x2C
#define REG_S2_HI_RES_DAC1	0x2D
#define REG_S3_HI_RES_DAC1	0x2E
#define REG_S4_HI_RES_DAC1	0x2D
#define REG_S5_HI_RES_DAC1	0x30
#define REG_S6_HI_RES_DAC1	0x31

//Die temperature
#define REG_DIE_TEMP_CONFIG		0x40
#define REG_DIE_TEMP_INT		  0x41
#define REG_DIE_TEMP_FRACTION	0x42

//DAC calibration
#define REG_DAC_CALIB_ENABLE	0x50

//SHA256
#define REG_SHA_CMD		  0xF0
#define REG_SHA_CONFIG	0xF1

//Memory
#define REG_MEMORY_CONTROL	0xF2
#define REG_MEMORY_INDEX	  0xF3
#define REG_MEMORY_DATA		  0xF4

// Part ID
#define REG_REV_ID		0xFE
#define REG_PART_ID		0xFF

#define REG_FIFO_DATA_MASK  0x07FFFF
#define REG_FIFO_RES        19
#define REG_FIFO_TAG_MASK   0x1F
