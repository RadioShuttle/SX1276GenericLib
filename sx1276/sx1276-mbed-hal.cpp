/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C) 2014 Semtech

Description: -

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainers: Miguel Luis, Gregory Cristian and Nicolas Huguenin
*/

/*
 * additional development to make it more generic across multiple OS versions
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */

#ifdef ARDUINO
 #include "arduino-mbed.h"
#endif

#include "sx1276-mbed-hal.h"



SX1276Generic::SX1276Generic( RadioEvents_t *events, BoardType_t board,
                            PinName mosi, PinName miso, PinName sclk, PinName nss, PinName reset,
                            PinName dio0, PinName dio1, PinName dio2, PinName dio3, PinName dio4, PinName dio5,
                            PinName antSwitch, PinName antSwitchTX, PinName antSwitchTXBoost, PinName tcxo)
                            : SX1276( events)
{
    Sleep_ms( 10 );
    this->RadioEvents = events;
    boardConnected = board;
    
    _antSwitchPwr = NULL;
    _antSwitch = NULL;
    _antSwitchTX = NULL;
    _antSwitchTXBoost = NULL;
    
    _tcxo = NULL;
    if (tcxo != NC)
        _tcxo = new DigitalOut(tcxo);
    
    switch(boardConnected) {
        case SX1276MB1MAS:
        case SX1276MB1LAS:
            _antSwitch = new DigitalOut(antSwitch);
            break;
        case RFM95_SX1276:
            break;
        case HELTEC_L4_1276:
            _antSwitchPwr = new DigitalOut(antSwitch);
            break;
        case MURATA_SX1276:
            _antSwitch = new DigitalOut(antSwitch);
            _antSwitchTX = new DigitalOut(antSwitchTX);
            _antSwitchTXBoost = new DigitalOut(antSwitchTXBoost);
            break;
        default:
            break;
    }
    _spi = new XSPI(mosi, miso, sclk );
    _nss = new DigitalOut(nss);
    
    _reset = new DigitalInOut(reset);
    
    _dio0 = NULL;
    _dio1 = NULL;
    _dio2 = NULL;
    _dio3 = NULL;
    _dio4 = NULL;
    _dio5 = NULL;
	if (dio0 != NC)
        _dio0 = new InterruptIn(dio0);
    if (dio1 != NC)
        _dio1 = new InterruptIn(dio1);
    if (dio2 != NC)
        _dio2 = new InterruptIn(dio2);
    if (dio3 != NC)
        _dio3 = new InterruptIn(dio3);
    if (dio4 != NC)
        _dio4 = new InterruptIn(dio4);
    if (dio5 != NC)
        _dio5 = new DigitalIn(dio5);
   
    Reset( );

    IoInit( );

    RxChainCalibration( );
 
    SetOpMode( RF_OPMODE_SLEEP );

    IoIrqInit( dioIrq );

    RadioRegistersInit( );

    SetModem( MODEM_FSK );
}

SX1276Generic::~SX1276Generic()
{
    txTimeoutTimer.detach();
    rxTimeoutTimer.detach();
    rxTimeoutSyncWord.detach();
    
    if (_antSwitchPwr)
        delete(_antSwitchPwr);
    if (_antSwitch)
    	delete _antSwitch;
    if (_antSwitchTX)
    	delete _antSwitchTX;
    if (_antSwitchTXBoost)
    	delete _antSwitchTXBoost;
    
    if (_tcxo) {
        *_tcxo = 0;
        delete (_tcxo);
    }
    /*
     * Reset(); // to put chip back into fresh state
     * We disabled the Reset() to allow the destructor to keep the
     * chip in Sleep() mode to stay turned off with little energy 
     * consumption.
     */
    delete _reset;
    delete _spi;
    delete _nss;
    
    if (_dio0)
        delete _dio0;
    if (_dio1)
    	delete _dio1;
    if (_dio2)
        delete _dio2;
    if (_dio3)
        delete _dio3;
    if (_dio4)
    	delete _dio4;
    if (_dio5)
    	delete _dio5;
}


//-------------------------------------------------------------------------
//                      Board relative functions
//-------------------------------------------------------------------------
uint8_t SX1276Generic::DetectBoardType( void )
{
    return boardConnected;
}

void SX1276Generic::IoInit( void )
{
    if (_tcxo)
        *_tcxo = 1;
    AntSwInit( );
    SpiInit( );
}


void SX1276Generic::SpiInit( void )
{
    *_nss = 1;
    _spi->format( 8,0 );
    uint32_t frequencyToSet = 8000000;
#ifdef TARGET_KL25Z	//busclock frequency is halved -> double the spi frequency to compensate
    _spi->frequency( frequencyToSet * 2 );
#else
    _spi->frequency( frequencyToSet );
#endif
    Sleep_ms(100);
}

void SX1276Generic::IoIrqInit( DioIrqHandler *irqHandlers )
{
    if (_dio0)
    	_dio0->rise(callback(this, static_cast< Trigger > ( irqHandlers[0] )));
    else
        irqHandlers[0] = NULL;
    
    if (_dio1)
    	_dio1->rise(callback(this, static_cast< Trigger > ( irqHandlers[1] )));
 	else
    	irqHandlers[1] = NULL;
    
    if (_dio2)
    	_dio2->rise(callback(this, static_cast< Trigger > ( irqHandlers[2] )));
    else
        irqHandlers[2] = NULL;
    
    if (_dio3)
    	_dio3->rise(callback(this, static_cast< Trigger > ( irqHandlers[3] )));
    else
        irqHandlers[3] = NULL;
    
    if (_dio4)
        _dio4->rise(callback(this, static_cast< Trigger > ( irqHandlers[4] )));
	else
        irqHandlers[4] = NULL;
}

void SX1276Generic::IoDeInit( void )
{
    //nothing
}

void SX1276Generic::SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;
    
    paConfig = Read( REG_PACONFIG );
    paDac = Read( REG_PADAC );
    
    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | GetPaSelect( this->settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
    
    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    Write( REG_PACONFIG, paConfig );
    Write( REG_PADAC, paDac );
}


uint8_t SX1276Generic::GetPaSelect( uint32_t channel )
{
    if( channel > RF_MID_BAND_THRESH )
    {
        if (boardConnected == SX1276MB1LAS || boardConnected == RFM95_SX1276 || boardConnected == MURATA_SX1276 || boardConnected == HELTEC_L4_1276)
        {
            return RF_PACONFIG_PASELECT_PABOOST;
        }
        else
        {
            return RF_PACONFIG_PASELECT_RFO;
        }
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276Generic::SetAntSwLowPower( bool status )
{
    if( isRadioActive != status )
    {
        isRadioActive = status;
    
        if( status == false )
        {
            AntSwInit( );
        }
        else
        {
            AntSwDeInit( );
        }
    }
}

void SX1276Generic::AntSwInit( void )
{
    if (_antSwitch)
    	*_antSwitch = 0;
    if (boardConnected == MURATA_SX1276) {
    	*_antSwitchTX = 0;
		*_antSwitchTXBoost = 0;
    }
    if (boardConnected == HELTEC_L4_1276)
        *_antSwitchPwr = PWR_OFF;
}

void SX1276Generic::AntSwDeInit( void )
{
    if (_antSwitch)
    	*_antSwitch = 0;
    if (boardConnected == MURATA_SX1276) {
        *_antSwitchTX = 0;
    	*_antSwitchTXBoost = 0;
    }
    if (boardConnected == HELTEC_L4_1276)
        *_antSwitchPwr = PWR_OFF;
}


void SX1276Generic::SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
        case RFLR_OPMODE_TRANSMITTER:
            if (boardConnected == MURATA_SX1276) {
	            *_antSwitch = 0;// Murata-RX
                if (Read( REG_PACONFIG) & RF_PACONFIG_PASELECT_PABOOST)
                    *_antSwitchTXBoost = 1;
            	else
               		*_antSwitchTX = 1; 	// alternate: antSwitchTXBoost = 1
            } else {
                if (_antSwitch)
	        		*_antSwitch = 1;
			}
            if (boardConnected == HELTEC_L4_1276)
                *_antSwitchPwr = PWR_ON;
            break;
        case RFLR_OPMODE_RECEIVER:
        case RFLR_OPMODE_RECEIVER_SINGLE:
        case RFLR_OPMODE_CAD:
            if (boardConnected == MURATA_SX1276) {
                *_antSwitch = 1;  // Murata-RX
            	*_antSwitchTX = 0;
            	*_antSwitchTXBoost = 0;
            } else if (boardConnected == HELTEC_L4_1276) {
                *_antSwitchPwr = PWR_ON;
    		} else {
                if (_antSwitch)
        			_antSwitch = 0;
            }
            break;
        case RFLR_OPMODE_SLEEP:
        case RFLR_OPMODE_STANDBY:
        default:
            if (boardConnected == MURATA_SX1276) {
                *_antSwitch = 0;  //Murata-RX
            	*_antSwitchTX = 0;
            	*_antSwitchTXBoost = 0;
            } else if (boardConnected == HELTEC_L4_1276) {
                *_antSwitchPwr = PWR_OFF;
    		} else {
                if (_antSwitch)
        			*_antSwitch = 0;
            }
            break;
    }
}

void SX1276Generic::SetTimeout(TimeoutTimer_t timer, timeoutFuncPtr func, int timeout_ms)
{
    switch(timer) {
	    case RXTimeoutTimer:
            if (func)
                rxTimeoutTimer.attach_us(callback(this, func), timeout_ms);
            else
                rxTimeoutTimer.detach();
            break;
        case TXTimeoutTimer:
            if (func)
                txTimeoutTimer.attach_us(callback(this, func), timeout_ms);
            else
                txTimeoutTimer.detach();
            break;
        case RXTimeoutSyncWordTimer:
            if (func)
                rxTimeoutSyncWord.attach_us(callback(this, func), timeout_ms);
            else
                rxTimeoutSyncWord.detach();
            break;
    }
}

void
SX1276Generic::Sleep_ms(int ms)
{
    wait_us(ms * 1000);
}

bool SX1276Generic::CheckRfFrequency( uint32_t frequency )
{
    if (frequency > 1200000)
        return false;
    // Implement check. Currently all frequencies are supported
    return true;
}

void SX1276Generic::Reset( void )
{
	_reset->output();
	*_reset = 0;
	Sleep_ms( 1 );
    *_reset = 1;
    _reset->input();	// I don't know why input again, maybe to save power (Helmut T)
	Sleep_ms( 6 );
}

void SX1276Generic::Write( uint8_t addr, uint8_t data )
{
    Write( addr, &data, 1 );
}

uint8_t SX1276Generic::Read( uint8_t addr )
{
    uint8_t data;
    Read( addr, &data, 1 );
    return data;
}

void SX1276Generic::Write( uint8_t addr, void *buffer, uint8_t size )
{
    uint8_t i;
    uint8_t *p = (uint8_t *)buffer;

    *_nss = 0; // what about SPI hold/release timing on fast MCUs? Helmut
    _spi->write( addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        _spi->write(*p++);
    }
    *_nss = 1;
}

void SX1276Generic::Read( uint8_t addr, void *buffer, uint8_t size )
{
    uint8_t i;
    uint8_t *p = (uint8_t *)buffer;
    
    *_nss = 0; // what about SPI hold/release timing on fast MCUs? Helmut
    _spi->write( addr & 0x7F );
    for( i = 0; i < size; i++ )
    {
        *p++ = _spi->write( 0 );
    }
    *_nss = 1;
}

void SX1276Generic::WriteFifo( void *buffer, uint8_t size )
{
    Write( 0, buffer, size );
}

void SX1276Generic::ReadFifo( void *buffer, uint8_t size )
{
    Read( 0, buffer, size );
}
