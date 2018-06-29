#define BAUDDEBUG 9600
#define BAUDPOS 9600
#define BAUDAQ 230400 //AQ required baud rate
#define MESSAGE_SIZE 100

#define DEFAULT_ACCURACY 350
#define DEFAULT_DOP 110

char receivedMessage[MESSAGE_SIZE];
boolean newMessage = false;
long latitude = 0;
long longitude = 0;
long altitude = 0;
long oldLatitude = 0;
long oldLongitude = 0;
long positionHack = 2;

int counter = 0;

//Initialize UBX messages
byte NAV_POSLLH[] = {0xB5, 0x62, 0x01, 0x02, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte NAV_DOP[] = {0xB5, 0x62, 0x01, 0x04, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void setup(){
  delay(10000);
  
  //Serial.begin(BAUDDEBUG); ///////DEBUG
  Serial1.begin(BAUDPOS);
  Serial2.begin(BAUDAQ);

  //Set default accuracies
  byteSplit(&NAV_POSLLH[26], DEFAULT_ACCURACY, 4);
  byteSplit(&NAV_POSLLH[30], DEFAULT_ACCURACY, 4);
  checksum(&NAV_POSLLH[2], sizeof(NAV_POSLLH) - 4);
  //Set default DOPs
  for(int i = 10; i < 24; i = i + 2){
    byteSplit(&NAV_DOP[i], DEFAULT_DOP, 2);
  }
  checksum(&NAV_DOP[2], sizeof(NAV_DOP) - 4);
}

void loop(){
  receiveMessage();
  parseNMEA();
}

////////////////////RECEIVING////////////////////
void receiveMessage(){
  static boolean receiving = false;
  static byte index = 0;
  char startChar = '$';
  char endChar = '\n';
  char readData;
  
  while(Serial1.available() > 0 && newMessage == false){
    readData = Serial1.read();
    if(receiving == true){
      if(readData != endChar){
        receivedMessage[index] = readData;
        index++;
        if(index >= MESSAGE_SIZE){
          index = MESSAGE_SIZE - 1;
        }
      }
      else{
        receivedMessage[index] = '\0';
        receiving = false;
        index = 0;
        newMessage = true;
      }
    }
    else if(readData == startChar){
      receiving = true;
    }
  }
}

void parseNMEA(){
  static char posMSG[] = "GGA";
  char temp[10] = {0};
  if(newMessage == true){
    //NMEAchecksumCalculator();
    char* strtokIndex;
    strtokIndex = strtok(receivedMessage, ",");
    if(posMSG[0] == strtokIndex[2] && posMSG[1] == strtokIndex[3] && posMSG[2] == strtokIndex[4]){
      parseGGAData(strtokIndex);
    }
    newMessage = false;
  }
}

void parseGGAData(char* index){
  //LATITUDE
  char ddBuffer[3];
  char mmBuffer[] = "000000000\0";
  index = strtok(NULL, ",");
  index = strtok(NULL, ",");
  if(strlen(index) > 1){
    ddBuffer[0] = index[0];
    ddBuffer[1] = index[1];
    ddBuffer[2] = '\0';
    int len = strlen(index);
    if(len > 10){
      len = 10;
    }
    int j = 0;
    for(int i = 0; i < len - 2; i++){
      if(index[i + 2] == '.'){
        j++;
      }
      else{
        mmBuffer[i - j] = index[i + 2];
      }
    }
    latitude = atol(mmBuffer) / 60 + atol(ddBuffer) * 10000000;
  
    //LONGITUDE
    char mmBuffer2[] = "000000000\0";
    index = strtok(NULL, ",");
    index = strtok(NULL, ",");
    ddBuffer[0] = index[1];
    ddBuffer[1] = index[2];
    ddBuffer[2] = '\0';
    len = strlen(index);
    if(len > 10){
      len = 10;
    }
    j = 0;
    for(int i = 0; i < len - 3; i++){
      if(index[i + 3] == '.'){
        j++;
      }
      else{
        mmBuffer2[i - j] = index[i + 3];
      }
    }
    longitude = atol(mmBuffer2) / 60 + atol(ddBuffer) * 10000000;

    //ALTITUDE
    char altBuffer[] = "0000000\0";
    for(int i = 0; i < 5; i++){
      index = strtok(NULL, ",");
    }
    j = 0;
    int k = 4;
    for(int i = strlen(index) - 1; i > -1; i--){
      if(index[i] != '.'){
        altBuffer[k] = index[i];
        k--;
      }
    }
    altitude = atol(altBuffer);

    //Workaround due to AQ's handling of a uBlox error, where the module sends the same position multiple times.
    if(latitude == oldLatitude && longitude == oldLongitude){
      positionHack = -positionHack;
      latitude += positionHack;
      oldLatitude = latitude;
    }
    else{
      oldLatitude = latitude;
      oldLongitude = longitude;
    }

    counter++;
    positionUpdate();
    sendUBXMessage(&NAV_POSLLH[0], sizeof(NAV_POSLLH));
    if(counter > 100){
      sendUBXMessage(&NAV_DOP[0], sizeof(NAV_DOP));
      counter = 0;
    }
  }
}

////////////////////TRANSMITTING////////////////////
//Takes a UBX message and the payload size and calculates the checksum for a UBX message
void checksum(byte *messagePayload, int payloadSize){
  byte CK_A = 0, CK_B = 0;
  for(int i = 0; i < payloadSize ;i++){
    CK_A = CK_A + *messagePayload;
    CK_B = CK_B + CK_A;
    messagePayload++;
  }
  *messagePayload = CK_A;
  messagePayload++;
  *messagePayload = CK_B;
}

//Takes a UBX message array, an integer value and a number of bytes and assigns the integer value to the number of bytes in little endian format
void byteSplit(byte *byteArray, long integerValue, int numberOfBytes){
  *byteArray = integerValue & 0xFF;
  for(int i = 8; i < numberOfBytes * 8; i = i + 8){
    byteArray++;
    *byteArray = (integerValue >> i) & 0xFF;
  }
}

//Takes latitude and longitude with a scaling factor of 1e7, altitude in millimeters and updates the UBX position message
void positionUpdate(){
  byteSplit(&NAV_POSLLH[10], longitude, 4);
  byteSplit(&NAV_POSLLH[14], latitude, 4);
  //byteSplit(&NAV_POSLLH[18], altitude, 4); //TEST IGNORING THE HEIGHT ABOVE ELLIPSOID
  byteSplit(&NAV_POSLLH[22], altitude, 4);
  checksum(&NAV_POSLLH[2], sizeof(NAV_POSLLH)-4);
}

//Takes a UBX message and its length and sends the message over serial
void sendUBXMessage(byte *UBXmessage, byte messageLength){
  for(int i = 0; i < messageLength; i++) {
    Serial2.write(UBXmessage[i]);
    Serial2.flush(); //Wait for data to be sent and buffer to clear
  }
}
