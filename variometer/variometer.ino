//nook: from white: GND-TX-RX-POWER

#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <inv_mpu.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <lightnmea.h>
#include <SSD1306_text.h>


/*!!!!!!!!!!!!!!!!!!!!!!!*/
/* VARIOMETER STRUCTURE  */
/*!!!!!!!!!!!!!!!!!!!!!!!*/
#define HAVE_SPEAKER
//#define HAVE_ACCELEROMETER
#define HAVE_SCREEN
//#define HAVE_GPS
#define HAVE_BT


// the variometer seems to be more stable at half speed
// don't hesitate to experiment
#if F_CPU >= 16000000L
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV4
#define SDCARD_SPEED SPI_CLOCK_DIV4
#else
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV2
#define SDCARD_SPEED SPI_CLOCK_DIV2
#endif //CPU_FREQ

/*!!!!!!!!!!!!!!!!!!!!!!!*/
/* VARIOMETER PARAMETERS */
/*!!!!!!!!!!!!!!!!!!!!!!!*/
#define VARIOMETER_SINKING_THRESHOLD -2.0
#define VARIOMETER_CLIMBING_THRESHOLD 0.3
#define VARIOMETER_NEAR_CLIMBING_SENSITIVITY 0.5

//#define VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM //TODO What is it??
//#define VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP  //TODO What is it??

/* mean filter duration = filter size * 2 seconds */
#define VARIOMETER_SPEED_FILTER_SIZE 5

/*****************/
/* screen objets */
/*****************/
#ifdef HAVE_SCREEN

#define OLED_DC     A4
#define OLED_CS     A3  //not use with 4wire spi (connected to GND), could be needed if more than one SPI device
#define OLED_RST  A5
SSD1306_text oled(OLED_DC, OLED_RST, OLED_CS);

#endif //HAVE_SCREEN

/**********************/
/* alti/vario objects */
/**********************/
#define POSITION_MEASURE_STANDARD_DEVIATION 0.1
#ifdef HAVE_ACCELEROMETER 
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.3
#else
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.6
#endif //HAVE_ACCELEROMETER 

kalmanvert kalmanvert;
#ifdef HAVE_SPEAKER
beeper beeper(VARIOMETER_SINKING_THRESHOLD, VARIOMETER_CLIMBING_THRESHOLD, VARIOMETER_NEAR_CLIMBING_SENSITIVITY);

#define FLIGHT_START_MIN_TIMESTAMP 15000
#define FLIGHT_START_VARIO_LOW_THRESHOLD (-0.5)
#define FLIGHT_START_VARIO_HIGH_THRESHOLD 0.5
#define FLIGHT_START_MIN_SPEED 10.0
boolean beepNearThermalEnabled = false;
int NewDataEnable = 0; //added to refresh screen and BT //TODO: usefull??
float altTakeoff;
float currentAlti;

#endif

/***************/
/* gps objects */
/***************/
#ifdef HAVE_GPS

#define GPS_CALIBRATION_STEPS 5

NmeaParser parser;
boolean gpsDataStarted = false;
boolean gpsAltiCalibrated = false;
unsigned char gpsAltiCalibrationStep = 0;

unsigned long speedFilterTimestamps[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterSpeedValues[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterAltiValues[VARIOMETER_SPEED_FILTER_SIZE];
int8_t speedFilterPos = 0;


#endif //HAVE_GPS


/***************/
/* BT objects */
/***************/
#ifdef HAVE_BT
String NMEA;
int CS;
float BT_vario;
#endif //HAVE_BT

/***************/
/* Battery objects */
/***************/
#define VBATPIN A9  //change pin to 10, 9 is taken by beeper tone AC library.
float measuredvbat = 0;

/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
   
Serial.begin(9600); //debug
//while(!Serial);        //debug wait for the serial monitor.
//Serial.println("setup start");//debug

/****************/
/* init BT  */
/****************/

#ifdef HAVE_BT
Serial1.begin(9600); //Serial1 is TX/RX on 32u4, Serial is for USB only
#endif //HAVE_BT




  /***************/
  /* init screen */
  /***************/
#ifdef HAVE_SCREEN

oled.init();
oled.clear();  
oled.setTextSize(2,2); // 2*(5x7) characters, pixel spacing = 2
oled.setCursor(0,0); // setCursor(row, pixel)    - sets write location to the specified row (0 to 7) and pixel (0 to 127)
oled.write("Alt: ");
oled.setCursor(2,0); // setCursor(row, pixel)    - sets write location to the specified row (0 to 7) and pixel (0 to 127)
oled.write("Alt0: ");

  
#endif //HAVE_SCREEN
  
  /************************************/
  /* init altimeter and accelerometer */
  /************************************/
  /* !!! fastwire don't take on account the cpu frequency !!! */
#if F_CPU >= 16000000L
  Fastwire::setup(400,0);
#else
  Fastwire::setup(800,0);
#endif //CPU_FREQ    
  ms5611_init();
#ifdef HAVE_ACCELEROMETER
  vertaccel_init();
#endif //HAVE_ACCELEROMETER

  /************/
  /* init gps */
  /************/
#ifdef HAVE_GPS
  Serial1.begin(9600);
  //Serial1.begin(38400);

#endif //HAVE_GPS
  
  /******************/
  /* get first data */
  /******************/
  /* wait for first alti and acceleration */
  while( ! (ms5611_dataReady()
#ifdef HAVE_ACCELEROMETER
            && vertaccel_dataReady()
#endif //HAVE_ACCELEROMETER
            ) ) {  //Serial.println("Waiting for first data");//debug
  }
  /* get first data */
  ms5611_updateData();

#ifdef HAVE_ACCELEROMETER
  vertaccel_updateData();
#endif //HAVE_ACCELEROMETER

  /* init kalman filter */
  kalmanvert.init(ms5611_getAltitude(),
#ifdef HAVE_ACCELEROMETER
                  vertaccel_getValue(),
#else
                  0.0,
#endif
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());

altTakeoff = ms5611_getAltitude(); //init altitude at takeoff  //TODO:do it better??
 
//Serial.println("Setup done"); //debug

}

/*----------------*/
/*      LOOP      */
/*----------------*/
  

void loop() {
 
  /*****************************/
  /* compute vertical velocity */
  /*****************************/
#ifdef HAVE_ACCELEROMETER
  if( ms5611_dataReady() && vertaccel_dataReady() ) {
    ms5611_updateData();
    vertaccel_updateData();

    kalmanvert.update( ms5611_getAltitude(),
                       vertaccel_getValue(),
                       millis() );
    NewDataEnable = 1; //for screen and BT update
                       
//    Serial.print("Alt:"); //DEBUG
//    Serial.print(ms5611_getAltitude());//DEBUG
//    Serial.print("  Vertaccel:"); //DEBUG
//    Serial.print(vertaccel_getValue());//DEBUG
//    Serial.print("  Vario:"); //DEBUG
//    Serial.println(kalmanvert.getVelocity());//DEBUG

#else
  if( ms5611_dataReady() ) {
    ms5611_updateData();

    kalmanvert.update( ms5611_getAltitude(),
                       0.0,
                       millis() );
    NewDataEnable = 1; //for screen and BT update

//    Serial.print("Alt:"); //DEBUG
//    Serial.print(ms5611_getAltitude());//DEBUG
//    Serial.print("  Vario:"); //DEBUG
//    Serial.println(kalmanvert.getVelocity());//DEBUG
//    Serial.print("  Pressure:"); //DEBUG
//    Serial.print(ms5611_getPressure());//DEBUG
//    Serial.print("   Température:"); //DEBUG
//    Serial.println(ms5611_getTemperature());//DEBUG



  /**************//**************//**************//**************//**************//**************//**************//**************/
  /**************/
  /* Send BT info to XCSoar with OpenVario protocol */ 
  /**************/
#ifdef HAVE_BT

  String string1 = "POV,E,"; //E: TE vario in m/s
  char string2[5];
  dtostrf(kalmanvert.getVelocity(),4,2,string2); //4:minimum width of output and 2 is number of decimals
  String string3 = ",P,";
  char string4[7];
  dtostrf(ms5611_getPressure(),6,2,string4); //6:minimum width of output and 2 is number of decimals  
  String string5 = ",T,";
  char string6[6];
  dtostrf(ms5611_getTemperature(),4,2,string6); //6:minimum width of output and 2 is number of decimals  
  
  
  String OpenVario = string1 + string2 + string3 + string4 + string5 + string6;
  
  int XOR = 0;
  int i = 0;
  // Calculate checksum ignoring any $'s in the string
  for (i = 0; i < OpenVario.length(); i++) {
      XOR ^= char(OpenVario[i]);
  }

  Serial1.print("$");
  Serial1.print(OpenVario);
  Serial1.print("*");
  Serial1.println(XOR,HEX);
//}

#endif //HAVE_BT
    
/**************//**************//**************//**************//**************//**************//**************//**************/
#endif //HAVE_ACCELEROMETER

    /* set beeper */
#ifdef HAVE_SPEAKER
    beeper.setVelocity( kalmanvert.getVelocity() );
#endif //HAVE_SPEAKER
  }

  /*****************/
  /* update beeper */
  /*****************/
#ifdef HAVE_SPEAKER
  beeper.update();

  /* check if near thermal features need to be started */
#if defined(VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM) || defined(VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP)
  if( !beepNearThermalEnabled ) {
    /* check flight start conditions */
    if( (millis() > FLIGHT_START_MIN_TIMESTAMP) &&
        (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD)
#ifdef HAVE_GPS
        && gpsAltiCalibrated && (parser.getSpeed() > FLIGHT_START_MIN_SPEED)
#endif //HAVE_GPS
      ) {
    beepNearThermalEnabled = true;
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
    beeper.setGlidingAlarmState(true);
#endif
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP
    beeper.setGlidingBeepState(true);
#endif
    }
  }  
#endif //defined(VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM) || defined(VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP)
#endif //HAVE_SPEAKER


  /**************/
  /* update battery  */
  /**************/
measuredvbat = analogRead(VBATPIN)*6.6/1024; 
//measuredvbat *= 2;    // we divided by 2, so multiply back
//measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
//measuredvbat /= 1024; // convert to voltage
//Serial.print("VBat: " ); Serial.println(measuredvbat); /debug
  

  /**************/
  /* update gps */
  /**************/  
#ifdef HAVE_GPS
  if( Serial1.available() > 0 ) {
  int inByte = Serial1.read();//debug
    Serial.write(inByte);//debug
    /* if needed wait for GPS initialization */
    if( ! gpsDataStarted ) {
      while (Serial1.available() > 0) {
        int c = Serial1.read();
        if( c == '$' ) {
          parser.getChar(c);
          gpsDataStarted = true;
          Serial.println("GPS data started"); //debug

        }
      }
    }
    
    /* else parse NMEA and save to sdcard */
    if( gpsDataStarted ) {
      while (Serial1.available() > 0) {
        int c = Serial1.read();
        parser.getChar(c);
        Serial.write(c); //debug

      }
    }
  }
  
  /* recalibrate alti with gps */
  if( ! gpsAltiCalibrated ) {
   if( parser.haveNewAltiValue() ) {
     gpsAltiCalibrationStep++;
     parser.getAlti(); //clear the new value flag
     if( gpsAltiCalibrationStep == GPS_CALIBRATION_STEPS) { //get 5 alti values before calibrating
       double gpsAlti = parser.getAlti();
       ms5611_setCurrentAltitude(gpsAlti);
       kalmanvert.resetPosition(gpsAlti);
       gpsAltiCalibrated = true;
     }
   }
  }
#endif //HAVE_GPS
  
  /*****************/
  /* update screen */
  /*****************/
#ifdef HAVE_SCREEN
  /* alternate display : alti / vario */
//  if( screenStatus == 0 ) {
//    altiDigit.display( kalmanvert.getPosition() );
//    screenStatus = 1;
//  } else {
//    varioDigit.display( kalmanvert.getVelocity() );
//    screenStatus = 0;
//  }
  if (NewDataEnable == 1) {
    //oled.clear();
    oled.setTextSize(2,2); // 2*(5x7) characters, pixel spacing = 2
    oled.setCursor(0,70); // setCursor(row, pixel)    - sets write location to the specified row (0 to 7) and pixel (0 to 127)
    oled.print(kalmanvert.getPosition(),0);




    float alt0 = kalmanvert.getPosition() - altTakeoff;

    if (alt0 >= 0) {
      oled.setCursor(2,57);
      oled.write(" "); //clear the minus sign (-) if velocity is positive
      oled.setCursor(2,69);
    }
    else {
      oled.setCursor(2,57);
    }    
    oled.print(alt0,0);

    oled.setTextSize(3,2);
    if (kalmanvert.getVelocity() >= 0) {
    oled.setCursor(5,0);
    oled.write(" "); //clear the minus sign (-) if velocity is positive
    oled.setCursor(5,17);
    
    }
    else {
      oled.setCursor(5,0);
    }
    
    oled.print(kalmanvert.getVelocity(),1); 

    oled.setTextSize(1,1);
    oled.setCursor(7,99);
    oled.print(measuredvbat); 
    
    NewDataEnable == 0; 
  }


  
#ifdef HAVE_GPS


 if ( parser.haveNewAltiValue()) {              //added
       Serial.print("GPSalti: ");//added
       Serial.println(parser.getAlti());//added
 }

  /* when getting speed from gps, display speed and ratio */
  if ( parser.haveNewSpeedValue() ) {

    /* get new values */
    unsigned long baseTime = speedFilterTimestamps[speedFilterPos];
    unsigned long deltaTime = millis(); //computed later
    speedFilterTimestamps[speedFilterPos] = deltaTime;
    
    double deltaAlti = speedFilterAltiValues[speedFilterPos]; //computed later
    speedFilterAltiValues[speedFilterPos] = kalmanvert.getPosition(); 

    double currentSpeed = parser.getSpeed();
    speedFilterSpeedValues[speedFilterPos] = currentSpeed;

    speedFilterPos++;
    if( speedFilterPos >= VARIOMETER_SPEED_FILTER_SIZE )
      speedFilterPos = 0;

    /* compute deltas */
    deltaAlti -= kalmanvert.getPosition();
    deltaTime -= baseTime;
    
    /* compute mean distance */
    double meanDistance = 0;
    int step = 0;
    while( step < VARIOMETER_SPEED_FILTER_SIZE ) {

      /* compute distance */
      unsigned long currentTime = speedFilterTimestamps[speedFilterPos];
      meanDistance += speedFilterSpeedValues[speedFilterPos] * (double)(currentTime - baseTime);
      baseTime = currentTime;

      /* next */
      speedFilterPos++;
      if( speedFilterPos >= VARIOMETER_SPEED_FILTER_SIZE )
        speedFilterPos = 0;
      step++;
    }

    /* compute glide ratio */
    double ratio = (meanDistance/3600.0)/deltaAlti;

    /* display speed and ratio */  
    Serial.print("GPSspeed: ");Serial.println(parser.getSpeed());//added  
//    //speedDigit.display( currentSpeed );    //TODO display speed on oled
//    if( currentSpeed >= RATIO_MIN_SPEED && ratio >= 0.0 && ratio < RATIO_MAX_VALUE ) {
//      ratioDigit.display(ratio);
//    } else {
//      ratioDigit.display(0.0);
//    }
  }
#endif //HAVE_GPS
#endif //HAVE_SCREEN 


}
