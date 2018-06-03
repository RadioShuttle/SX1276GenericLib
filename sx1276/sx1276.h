/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech

Description: Actual implementation of a SX1276 radio, inherits Radio

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/

/*
 * additional development to make it more generic across multiple OS versions
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */

#ifndef __SX1276_H__
#define __SX1276_H__

#include "radio.h"
#include "sx1276Regs-Fsk.h"
#include "sx1276Regs-LoRa.h"



/*!
 * Radio wake-up time from sleep
 */
#define RADIO_WAKEUP_TIME                           1 // [ms]

/*!
 * Sync word for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12

/*!
 * Sync word for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD                    0x34


/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

#define RX_BUFFER_SIZE                              256

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164.0
#define RSSI_OFFSET_HF                              -157.0

#define RF_MID_BAND_THRESH                          525000000




/*!
 * Type of the supported board. [SX1276MB1MAS / SX1276MB1LAS]
 */
typedef enum BoardType
{
    SX1276MB1MAS = 0,
    SX1276MB1LAS,
    RFM95_SX1276,
    MURATA_SX1276,
    HELTEC_L4_1276,
    UNKNOWN
}BoardType_t;


typedef enum {
    LORA_SF6 =  6,  //   64 chips/symbol, SF6 requires an TCXO!
    LORA_SF7 =  7,  //  128 chips/symbol
    LORA_SF8 =  8,  //  256 chips/symbol
    LORA_SF9 =  9,  //  512 chips/symbol
    LORA_SF10 = 10, // 1024 chips/symbol
    LORA_SF11 = 11, // 2048 chips/symbol
    LORA_SF12 = 12, // 4096 chips/symbol
} lora_spreading_factor_t;


typedef enum {  // cyclic error coding to perform forward error detection and correction
    LORA_ERROR_CODING_RATE_4_5 = 1,   // 1.25x overhead
    LORA_ERROR_CODING_RATE_4_6 = 2,   // 1.50x overhead
    LORA_ERROR_CODING_RATE_4_7 = 3,   // 1.75x overhead
    LORA_ERROR_CODING_RATE_4_8 = 4,   // 2.00x overhead
} lora_coding_rate_t;


typedef enum {
    RF_FREQUENCY_868_0 = 868000000, // Hz
    RF_FREQUENCY_868_1 = 868100000, // Hz
    RF_FREQUENCY_868_3 = 868300000, // Hz
    RF_FREQUENCY_868_5 = 868500000, // Hz
} rf_frequency_t;



/*!
 * Actual implementation of a SX1276 radio, inherits Radio
 */
class SX1276 : public Radio
{
protected:

    bool isRadioActive;

    BoardType_t boardConnected; //1 = SX1276MB1LAS; 0 = SX1276MB1MAS

    uint8_t *rxtxBuffer;
    
    /*!
     * Hardware IO IRQ callback function definition
     */
    typedef void ( SX1276::*DioIrqHandler )( void );

    /*!
     * Hardware DIO IRQ functions
     */
    DioIrqHandler *dioIrq;



    RadioSettings_t settings;

    /*!
     * FSK bandwidth definition
     */
    struct BandwidthMap {
        uint32_t bandwidth;
        uint8_t  RegValue;
    };
    static const struct BandwidthMap FskBandwidths[];
    static const struct BandwidthMap LoRaBandwidths[];
    
protected:

    /*!
    * Performs the Rx chain calibration for LF and HF bands
    * \remark Must be called just after the reset so all registers are at their
    *         default values
    */
    void RxChainCalibration( void );

public:
    SX1276( RadioEvents_t *events);
    virtual ~SX1276( );
    
    
    
    
    //-------------------------------------------------------------------------
    //                        Redefined Radio functions
    //-------------------------------------------------------------------------
    /*!
     * @brief Return current radio status, returns true if a radios has been found.
     *
     * @param [IN] events Structure containing the driver callback functions
     */
    virtual bool Init( RadioEvents_t *events );

    /*!
     *  @brief Initializes the radio registers
     */
    virtual void RadioRegistersInit(void);

    /*!
     * Return current radio status
     *
     * @param status Radio status. [RF_IDLE, RX_RUNNING, TX_RUNNING, CAD_RUNNING]
     */
    virtual RadioState GetStatus( void ); 

    /*!
     * @brief Configures the SX1276 with the given modem
     *
     * @param [IN] modem Modem to be used [0: FSK, 1: LoRa] 
     */
    virtual void SetModem( RadioModems_t modem );

    /*!
     * @brief Sets the channel frequency
     *
     * @param [IN] freq         Channel RF frequency
     */
    virtual void SetChannel( uint32_t freq );

    /*!
     * @brief Sets the channels configuration
     *
     * @param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
     * @param [IN] freq       Channel RF frequency
     * @param [IN] rssiThresh RSSI threshold
     *
     * @retval isFree         [true: Channel is free, false: Channel is not free]
     */
    virtual bool IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh );

    /*!
     * @brief Generates a 32 bits random value based on the RSSI readings
     *
     * \remark This function sets the radio in LoRa modem mode and disables
     *         all interrupts.
     *         After calling this function either Radio.SetRxConfig or
     *         Radio.SetTxConfig functions must be called.
     *
     * @retval randomValue    32 bits random value
     */
    virtual uint32_t Random( void );

    /*!
     * @brief Sets the reception parameters
     *
     * @param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
     * @param [IN] bandwidth    Sets the bandwidth
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     * @param [IN] datarate     Sets the Datarate
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     * @param [IN] coderate     Sets the coding rate ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     * @param [IN] bandwidthAfc Sets the AFC Bandwidth ( FSK only )
     *                          FSK : >= 2600 and <= 250000 Hz
     *                          LoRa: N/A ( set to 0 )
     * @param [IN] preambleLen  Sets the Preamble length ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: Length in symbols ( the hardware adds 4 more symbols )
     * @param [IN] symbTimeout  Sets the RxSingle timeout value
     *                          FSK : timeout number of bytes
     *                          LoRa: timeout in symbols
     * @param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
     * @param [IN] payloadLen   Sets payload length when fixed lenght is used
     * @param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
     * @param [IN] freqHopOn    Enables disables the intra-packet frequency hopping  [0: OFF, 1: ON] (LoRa only)
     * @param [IN] hopPeriod    Number of symbols bewteen each hop (LoRa only)
     * @param [IN] iqInverted   Inverts IQ signals ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     * @param [IN] rxContinuous Sets the reception in continuous mode
     *                          [false: single mode, true: continuous mode]
     */
    virtual void SetRxConfig ( RadioModems_t modem, uint32_t bandwidth,
                               uint32_t datarate, uint8_t coderate,
                               uint32_t bandwidthAfc, uint16_t preambleLen,
                               uint16_t symbTimeout, bool fixLen,
                               uint8_t payloadLen,
                               bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                               bool iqInverted, bool rxContinuous );

    /*!
     * @brief Sets the transmission parameters
     *
     * @param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
     * @param [IN] power        Sets the output power [dBm]
     * @param [IN] fdev         Sets the frequency deviation ( FSK only )
     *                          FSK : [Hz]
     *                          LoRa: 0
     * @param [IN] bandwidth    Sets the bandwidth ( LoRa only )
     *                          FSK : 0
     *                          LoRa: [0: 125 kHz, 1: 250 kHz,
     *                                 2: 500 kHz, 3: Reserved]
     * @param [IN] datarate     Sets the Datarate
     *                          FSK : 600..300000 bits/s
     *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
     *                                10: 1024, 11: 2048, 12: 4096  chips]
     * @param [IN] coderate     Sets the coding rate ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
     * @param [IN] preambleLen  Sets the preamble length
     * @param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
     * @param [IN] crcOn        Enables disables the CRC [0: OFF, 1: ON]
     * @param [IN] freqHopOn    Enables disables the intra-packet frequency hopping  [0: OFF, 1: ON] (LoRa only)
     * @param [IN] hopPeriod    Number of symbols bewteen each hop (LoRa only)
     * @param [IN] iqInverted   Inverts IQ signals ( LoRa only )
     *                          FSK : N/A ( set to 0 )
     *                          LoRa: [0: not inverted, 1: inverted]
     * @param [IN] timeout      Transmission timeout [ms]
     */
    virtual void SetTxConfig( RadioModems_t modem, int8_t power, uint32_t fdev,
                              uint32_t bandwidth, uint32_t datarate,
                              uint8_t coderate, uint16_t preambleLen,
                              bool fixLen, bool crcOn, bool freqHopOn,
                              uint8_t hopPeriod, bool iqInverted, uint32_t timeout );

    
    /*!
     * @brief Checks if the given RF frequency is supported by the hardware
     *
     * @param [IN] frequency RF frequency to be checked
     * @retval isSupported [true: supported, false: unsupported]
     */
    virtual bool CheckRfFrequency( uint32_t frequency ) = 0;
    
    /*!
     * @brief Computes the packet time on air for the given payload
     *
     * \Remark Can only be called once SetRxConfig or SetTxConfig have been called
     *
     * @param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
     * @param [IN] pktLen     Packet payload length
     *
     * @retval airTime        Computed airTime for the given packet payload length
     */
    virtual uint32_t TimeOnAir ( RadioModems_t modem, int16_t pktLen );

    /*!
     * @brief Sends the buffer of size. Prepares the packet to be sent and sets
     *        the radio in transmission
     *
     * @param [IN]: buffer     Buffer pointer
     * @param [IN]: size       Buffer size
     * @param [IN]: buffer     Header pointer
     * @param [IN]: size       Header size
     */
    virtual void Send(void *buffer, int16_t size, void *header = NULL, int16_t hsize = 0);

    /*!
     * @brief Sets the radio in sleep mode
     */
    virtual void Sleep( void );
    
    /*!
     * @brief Sets the radio in standby mode
     */
    virtual void Standby( void );
    
    /*!
     * @brief Sets the radio in CAD mode
     */
    virtual void StartCad( void );

    /*!
     * @brief Sets the radio in reception mode for the given time
     * @param [IN] timeout Reception timeout [ms]
     *                     [0: continuous, others timeout]
     */
    virtual void Rx( uint32_t timeout );
    
    /*!
     * @brief Check is radio receives a signal
     */
    virtual bool RxSignalPending();


    /*!
     * @brief Sets the radio in transmission mode for the given time
     * @param [IN] timeout Transmission timeout [ms]
     *                     [0: continuous, others timeout]
     */
    virtual void Tx( uint32_t timeout );

    /*!
     * @brief Sets the radio in continuous wave transmission mode
     *
     * @param [IN]: freq       Channel RF frequency
     * @param [IN]: power      Sets the output power [dBm]
     * @param [IN]: time       Transmission mode timeout [s]
     */

    virtual void SetTxContinuousWave( uint32_t freq, int8_t power, uint16_t time );

    /*!
     * @brief Returns the maximal transfer unit for a given modem
     *
     * @retval MTU size in bytes
     */
    virtual int16_t MaxMTUSize( RadioModems_t modem );
    
    /*!
     * @brief Reads the current RSSI value
     *
     * @retval rssiValue Current RSSI value in [dBm]
     */
    virtual int16_t GetRssi ( RadioModems_t modem );
    
    /*!
     * @brief Reads the current frequency error
     *
     * @retval frequency error value in [Hz]
     */
    virtual int32_t GetFrequencyError( RadioModems_t modem );
    
    /*!
     * @brief Writes the radio register at the specified address
     *
     * @param [IN]: addr Register address
     * @param [IN]: data New register value
     */
    virtual void Write ( uint8_t addr, uint8_t data ) = 0;

    /*!
     * @brief Reads the radio register at the specified address
     *
     * @param [IN]: addr Register address
     * @retval data Register value
     */
    virtual uint8_t Read ( uint8_t addr ) = 0;

    /*!
     * @brief Writes multiple radio registers starting at address
     *
     * @param [IN] addr   First Radio register address
     * @param [IN] buffer Buffer containing the new register's values
     * @param [IN] size   Number of registers to be written
     */
    virtual void Write( uint8_t addr, void *buffer, uint8_t size ) = 0;

    /*!
     * @brief Reads multiple radio registers starting at address
     *
     * @param [IN] addr First Radio register address
     * @param [OUT] buffer Buffer where to copy the registers data
     * @param [IN] size Number of registers to be read
     */
    virtual void Read ( uint8_t addr, void *buffer, uint8_t size ) = 0;

    /*!
     * @brief Writes the buffer contents to the SX1276 FIFO
     *
     * @param [IN] buffer Buffer containing data to be put on the FIFO.
     * @param [IN] size Number of bytes to be written to the FIFO
     */
    virtual void WriteFifo( void *buffer, uint8_t size ) = 0;

    /*!
     * @brief Reads the contents of the SX1276 FIFO
     *
     * @param [OUT] buffer Buffer where to copy the FIFO read data.
     * @param [IN] size Number of bytes to be read from the FIFO
     */
    virtual void ReadFifo( void *buffer, uint8_t size ) = 0;
    /*!
     * @brief Resets the SX1276
     */
    virtual void Reset( void ) = 0;

    /*!
     * @brief Sets the maximum payload length.
     *
     * @param [IN] modem      Radio modem to be used [0: FSK, 1: LoRa]
     * @param [IN] max        Maximum payload length in bytes
     */
    virtual void SetMaxPayloadLength( RadioModems_t modem, uint8_t max );

    /*!
     * \brief Sets the network to public or private. Updates the sync byte.
     *
     * \remark Applies to LoRa modem only
     *
     * \param [IN] enable if true, it enables a public network
     */
    virtual void SetPublicNetwork( bool enable );

    /*!
     * @brief Sets the radio output power.
     *
     * @param [IN] power Sets the RF output power
     */
    virtual void SetRfTxPower( int8_t power ) = 0;
    
    //-------------------------------------------------------------------------
    //                        Board relative functions
    //-------------------------------------------------------------------------
    /*!
     * Radio registers definition
     */
    struct RadioRegisters {
        ModemType   Modem;
        uint8_t     Addr;
        uint8_t     Value;
    };
    
    
    static const struct RadioRegisters RadioRegsInit[];

    typedef enum {
        RXTimeoutTimer,
        TXTimeoutTimer,
        RXTimeoutSyncWordTimer
    } TimeoutTimer_t;


protected:
    /*!
     * @brief Initializes the radio I/Os pins interface
     */
    virtual void IoInit( void ) = 0;
    
    /*!
     * @brief Initializes the radio SPI
     */
    virtual void SpiInit( void ) = 0;

    /*!
     * @brief Initializes DIO IRQ handlers
     *
     * @param [IN] irqHandlers Array containing the IRQ callback functions
     */
    virtual void IoIrqInit( DioIrqHandler *irqHandlers ) = 0;

    /*!
     * @brief De-initializes the radio I/Os pins interface. 
     *
     * \remark Useful when going in MCU lowpower modes
     */
    virtual void IoDeInit( void ) = 0;

    /*!
     * @brief Gets the board PA selection configuration
     *
     * @param [IN] channel Channel frequency in Hz
     * @retval PaSelect RegPaConfig PaSelect value
     */
    virtual uint8_t GetPaSelect( uint32_t channel ) = 0;

    /*!
     * @brief Set the RF Switch I/Os pins in Low Power mode
     *
     * @param [IN] status enable or disable
     */
    virtual void SetAntSwLowPower( bool status ) = 0;

    /*!
     * @brief Initializes the RF Switch I/Os pins interface
     */
    virtual void AntSwInit( void ) = 0;

    /*!
     * @brief De-initializes the RF Switch I/Os pins interface 
     *
     * \remark Needed to decrease the power consumption in MCU lowpower modes
     */
    virtual void AntSwDeInit( void ) = 0;

    /*!
     * @brief Controls the antenna switch if necessary.
     *
     * \remark see errata note
     *
     * @param [IN] opMode Current radio operating mode
     */
    virtual void SetAntSw( uint8_t opMode ) = 0;
    
    typedef void ( SX1276::*timeoutFuncPtr)( void );
    
    
    /*
     * The the Timeout for a given Timer.
     */
    virtual void SetTimeout(TimeoutTimer_t timer, timeoutFuncPtr, int timeout_ms = 0) = 0;
    
    /*
     * A simple ms sleep
     */
    virtual void Sleep_ms(int ms) = 0;

protected:

    /*!
     * @brief Sets the SX1276 operating mode
     *
     * @param [IN] opMode New operating mode
     */
    virtual void SetOpMode( uint8_t opMode );

    /*
     * SX1276 DIO IRQ callback functions prototype
     */

    /*!
     * @brief DIO 0 IRQ callback
     */
    virtual void OnDio0Irq( void );

    /*!
     * @brief DIO 1 IRQ callback
     */
    virtual void OnDio1Irq( void );

    /*!
     * @brief DIO 2 IRQ callback
     */
    virtual void OnDio2Irq( void );

    /*!
     * @brief DIO 3 IRQ callback
     */
    virtual void OnDio3Irq( void );

    /*!
     * @brief DIO 4 IRQ callback
     */
    virtual void OnDio4Irq( void );

    /*!
     * @brief DIO 5 IRQ callback
     */
    virtual void OnDio5Irq( void );

    /*!
     * @brief Tx & Rx timeout timer callback
     */
    virtual void OnTimeoutIrq( void );

    /*!
     * Returns the known FSK bandwidth registers value
     *
     * \param [IN] bandwidth Bandwidth value in Hz
     * \retval regValue Bandwidth register value.
     */
    static uint8_t GetFskBandwidthRegValue( uint32_t bandwidth );

    static uint8_t GetLoRaBandwidthRegValue( uint32_t bandwidth );
    
    enum {
        LORA_BANKWIDTH_7kHz  = 0, //  7.8 kHz requires TCXO
        LORA_BANKWIDTH_10kHz = 1, // 10.4 kHz requires TCXO
        LORA_BANKWIDTH_15kHz = 2, // 15.6 kHz requires TCXO
        LORA_BANKWIDTH_20kHz = 3, // 20.8 kHz requires TCXO
        LORA_BANKWIDTH_31kHz = 4, // 31.2 kHz requires TCXO
        LORA_BANKWIDTH_41kHz = 5, // 41.4 kHz requires TCXO
        LORA_BANKWIDTH_62kHz = 6, // 62.5 kHz requires TCXO
        LORA_BANKWIDTH_125kHz = 7,
        LORA_BANKWIDTH_250kHz = 8,
        LORA_BANKWIDTH_500kHz = 9,
        LORA_BANKWIDTH_RESERVED = 10,
    };
};

#endif // __SX1276_H__
