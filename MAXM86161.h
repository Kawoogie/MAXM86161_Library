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
// #ifdef __MAXM86161_H_
// #define __MAXM86161_H_


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
    /** @brief Initialization of the object */
    int init(void);
    /** @brief Start collecting data samples */
    int start(void);
    /** @brief Stop collecting data samples */
    int stop(void);

    // Configuration adjustments
    /** @brief Set the rate of the PPG sensor */
    int set_interrogation_rate(int rate);
    /** @brief Set the number of samples to average */
    int set_sample_averaging(int average);
    /** @brief Set the LED current for all LEDs */
    int set_all_led_current(int current);
    /** @brief Set the LED current for LED1 */   
    int set_led1_current(int current);
    /** @brief Set the LED current for LED2 */   
    int set_led2_current(int current);
    /** @brief Set the LED current for LED3 */   
    int set_led3_current(int current);
    /** @brief Set the integration time for the photodiode */   
    int set_ppg_tint(int time);

    // Setting adjustments
    int alc_on(void);
    int alc_off(void);
    int picket_on(void);
    int picket_off(void);


    // void read_fifo(int* red, int* green, int* ir);
    // void set_led_current(char brightness);
    // void set_led_red_current(char brightness);
    // void set_led_green_current(char brightness);
    // void set_led_ir_current(char brightness);
    // char read_package_temp();

    // Functions not yet working
    // char read_status();




    private:
    I2C & _i2cbus;

    // void _set_led_sequence(char sequence);

    int _read_from_reg(int address, int &data);
    int _write_to_reg(int address, int value);
    int _set_one_bit(int current_bits, int position);
    int _clear_one_bit(int current_bits, int position);

};



// I2C address of the PPG sensor
#define PPG_ADDR 0xC4

// Part ID of the MAXM86161
#define PPG_PART_ID 0x36


// Bit positions.
#define POS_START_STOP  1  

#define POS_ALC_DIS  7
#define POS_PPG_TINT 0

#define POS_PICKET_DIS  7

#define POS_PPG_SR 3
#define POS_SMP_AVG 0


// Bit masks.
#define MASK_SMP_AVE 0b00000111  //Register 0x12
#define MASK_PPG_SR 0b11111000  // Register 0x12
#define MASK_PPG_TINT_WRITE 0b11111100  // Register 0x11
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


/*******************************************************************************
 ************************** Maxm86161 Value Enum *******************************
 ******************************************************************************/

// enum sample_rates {
//     24.995 = 0, 50.027 = 1, 84.021 = 2, 99.902 = 3, 199.805 = 4, 399.610 = 5,
//                     8.0 = 0x0A, 16.0 = 0x0B, 32.0 = 0x0C, 64.0 = 0x0D, 128.0 = 0x0E,
//                     256.0 = 0x0F, 512.0 = 0x10, 1024.0 = 0x11, 2048.0 = 0x12, 4096.0 = 0x13,
//                     8 = 0x0A, 16 = 0x0B, 32 = 0x0C, 64 = 0x0D, 128 = 0x0E,
//                     256 = 0x0F, 512 = 0x10, 1024 = 0x11, 2048 = 0x12, 4096 = 0x13};


// #endif /* __MAXM86161_H_*/