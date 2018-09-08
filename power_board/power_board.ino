//arduino code

static int r = 5;
static int g = 3;
static int b = 6;
bool COUNTER = 0;

//Fehler Meldungen
static String lowBATTERY = "<Battery low>\n";
static String noBATTERY = "<Battery missing>\n";



void setup() {
  Serial.begin(9600);
  
  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
  
  analogWrite(r, 255);
  analogWrite(g, 255);
  analogWrite(b, 255);
  int i = 0;
  for(i; i<3;i++) {
    analogWrite(r, 235);
    analogWrite(g, 249);
    analogWrite(b, 249);
    
    delay(200);
    
    analogWrite(r, 255);
    analogWrite(g, 255);
    analogWrite(b, 255);

    delay(200);
  }

    Serial.println(" ______    _____");                         
    Serial.println("|  __  | /   ___|"); 
    Serial.println("| |  | | |  /   |");
    Serial.println("| |__| | |___/  |");
    Serial.println("ROVER__brunoOS_/");
    Serial.println();
    Serial.println("    help/info");
    //Serial.println("\n  <system ok>\n");

  float AKKU = VOLTAGE_CHECK(3, 4.7, 33, 0, 0, 0);
  if(AKKU < 25.9) {
          analogWrite(r, 220);
          if(COUNTER == 0){
            if(AKKU < 20.0) Serial.println(noBATTERY);
            else {
              Serial.println(lowBATTERY);
            }
          }
          COUNTER = 1;
        } else {
          //analogWrite(r, 255);
          COUNTER = 0;
        }
  Serial.print("\n> ");

}

void loop() {
  float TEMP_M, AKKU;
  while(Serial.available() > 0) {
    //menü
      String READ_DATA;//[LINE_BUF_SIZE];
      READ_DATA = Serial.readString();
      Serial.println(READ_DATA+"\n");

      String READ_DATA_CLEAR = trimSPACE(READ_DATA);
      //Serial.println(READ_DATA_CLEAR);
      
      int INDEX_1 = READ_DATA_CLEAR.indexOf('-');
      int INDEX_2 = READ_DATA_CLEAR.indexOf('-', INDEX_1 + 1);
      int INDEX_3 = READ_DATA_CLEAR.indexOf('-', INDEX_2 + 1);
      //Serial.println(READ_DATA);
      
      int PARAMETER_1 = 0;
      int PARAMETER_2 = 0;
      int PARAMETER_3 = 0;
      
      String COMMAND = READ_DATA_CLEAR.substring(0,INDEX_1);
      
      PARAMETER_1 = (READ_DATA_CLEAR.substring(INDEX_1 + 1, INDEX_2)).toInt();
      PARAMETER_2 = (READ_DATA_CLEAR.substring(INDEX_2 + 1, INDEX_3)).toInt();
      PARAMETER_3 = (READ_DATA_CLEAR.substring(INDEX_3 + 1)).toInt();
      
      //Serial.println(COMMAND + PARAMETER_1);

        if(COMMAND == "temp") {
          //Serial.println("temp");
       TEMP_M = TEMP_READ();
       //Serial.write(_2_7);//new page
       Serial.print("  Temperatur: "),Serial.print(TEMP_M), Serial.println("°C");
       }

        //Akku PWR
       else if(COMMAND == "voltagebattery") {
        //Serial.print("voltage battery -"), Serial.println(PARAMETER_1);
          AKKU = VOLTAGE_CHECK(3, 4.7, 33, 0, 0, PARAMETER_1);
          //Serial.print("  Spannung: "),Serial.print(AKKU), Serial.println("V ");
       }

        //Spannungsmessung Quest mit Ausgleichsgreade bezogen auf Fluke (WARR Werkstatt)
        else if(COMMAND == "voltagequest") {
          //Serial.print("voltage quest -"), Serial.println(PARAMETER_1);
          AKKU = VOLTAGE_CHECK(0, 4.7, 22, 0.0111, -0.0787, PARAMETER_1);
          if(PARAMETER_1 && (AKKU < 1.0 || AKKU > 24.0)) Serial.println("<Warnung>: Nur zwischen 1V und 24V genau!");
        }

        else if(COMMAND == "voltagenuc") {
        //Serial.print("voltage INTEL-NUC -"), Serial.println(PARAMETER_1);
          AKKU = VOLTAGE_CHECK(4, 4.7, 12, 0, 0, PARAMETER_1);
          //Serial.print("  Spannung: "),Serial.print(AKKU), Serial.println("V ");
       }

       else if(COMMAND == "voltagestable") {
        //Serial.print("voltage battery -"), Serial.println(PARAMETER_1);
          AKKU = VOLTAGE_CHECK(1, 4.7, 33, 0, 0, PARAMETER_1);
          //Serial.print("  Spannung: "),Serial.print(AKKU), Serial.println("V ");
       }

        else if(COMMAND == "ledon") {
          //Serial.println("led on");
          if(PARAMETER_1 == NULL) PARAMETER_1 = 35;
          if(PARAMETER_2 == NULL) PARAMETER_2 = 6;
          if(PARAMETER_3 == NULL) PARAMETER_3 = 6;
          
        analogWrite(r, 255-PARAMETER_1);
        analogWrite(g, 255-PARAMETER_2);
        analogWrite(b, 255-PARAMETER_3);
        
        PARAMETER_1 = NULL;
        PARAMETER_2 = NULL;
        PARAMETER_3 = NULL;
        }
        
        else if(COMMAND == "ledoff") {
          //Serial.println("led off");
        analogWrite(r, 255);
        analogWrite(g, 255);
        analogWrite(b, 255);
        }
        
        else if(COMMAND == "red") {
          //Serial.println("red");
        if(PARAMETER_1 == NULL) PARAMETER_1 = 35;
        analogWrite(r, 255-PARAMETER_1);
        PARAMETER_1 = NULL;
        }
        
        else if(COMMAND == "green") {
          //Serial.println("green");
        if(PARAMETER_1 == NULL) PARAMETER_1 = 6;
        analogWrite(g, 255-PARAMETER_1);
        PARAMETER_1 = NULL;
        }
        
        else if(COMMAND == "blue") {
          //Serial.println("blue");
        if(PARAMETER_1 == NULL) PARAMETER_1 = 6;
        analogWrite(g, 255-PARAMETER_1);
        PARAMETER_1 = NULL;
        }
        

        else if(COMMAND == "help") {
          //Serial.println("Help");
          Serial.println("Interface clear clr");
          Serial.println("Temperatur PCB  temp");
          Serial.println("\nSpannung:");
          Serial.println("Akku-PWR ------ voltage battery -VERBOUS[0/1]");
          Serial.println("\"voltage Quest\" voltage quest   -VERBOUS[0/1]");
          Serial.println("NUC ----------- voltage nuc     -VERBOUS[0/1]");
          Serial.println("geregelt ------ voltage stable  -VERBOUS[0/1]");
          Serial.println("Default: VERBOUS[0]");
          Serial.println("\nLED:");
          Serial.println("ein  led on -r[0-255] -g[0-255] -b[0-255]");
          Serial.println("rot  red    -r[0-255]");
          Serial.println("grün green  -g[0-255]");
          Serial.println("blau blue   -b[0-255]");
          Serial.println("aus  led off");
          Serial.println("Default: r[35] g[6] b[6]");
        }

       else if(COMMAND == "info") {
        //Serial.println("info");
          Serial.println("Software: Version 1.0 Beta");
          Serial.println("PCB:      Version 2.0");
          Serial.println("In Betrieb:");
          Serial.println("- Alle Spg und Temp");
          Serial.println("- Betriebsspannung für den NUC");
          Serial.println("- Betriebsspannung Logik");
       }
       else if(COMMAND == "clr") {
        for (int m = 0; m < 60; m++){
          Serial.println();
        }
       }        

        else {
          //Serial.println(READ_DATA);
          Serial.println("  <command not found> see help");
        }
    

        Serial.print("\n> ");
        //checks
        AKKU = VOLTAGE_CHECK(3, 4.7, 22, 0, 0, 0);
        //Serial.print("Spannung: "),Serial.print(AKKU), Serial.println("V");

        if(AKKU < 25.9) {
          analogWrite(r, 220);
          if(COUNTER == 0){
            if(AKKU < 20.0) Serial.println(noBATTERY);
            else {
              //Serial.println(lowBATTERY);
              Serial.println(lowBATTERY);
            }
          }
          COUNTER = 1;
        } else {
          //analogWrite(r, 255);
          COUNTER = 0;
        }
      }

}

String trimSPACE(String COMMAND) {
  String WO_SPACE; 
  int length = COMMAND.length(), i;
  for(i = 0; i< length; i++) {
    if(COMMAND[i] != ' ') WO_SPACE += COMMAND[i];
  }

  return WO_SPACE;
}

//Temperaturmessung
float TEMP_READ(){
  
  float TEMP_M = 0;
  int I = 1;
  for(;I<=10;I++) {
    TEMP_M = TEMP_M + (analogRead(A2)*(5.0/1023)-0.5)/0.01;
  }
  TEMP_M = TEMP_M/(I-1);
  //TEMP_M = TEMP_M/I;
  return(TEMP_M);
}

//Spannungsmessung
float VOLTAGE_CHECK(int PORT, float Rm, float Rv, float a, float b, int VERBOUS) {
  if(VERBOUS) Serial.print("Port A"),Serial.println(PORT);
  float SPG = analogRead(PORT)*(5.0/1023)*((Rm + Rv)/Rm);//*8.02;
  if(VERBOUS) Serial.print("U= "),Serial.print(SPG), Serial.println("V");
  
  float error = a*SPG + b; //Fehler mit den gegebenen Paametern berechnen
  SPG = SPG - error; //Fehler in die Spannung einrechnen
  
  if(VERBOUS && error != 0) Serial.print("e= "),Serial.print(error), Serial.print("V");
  if(VERBOUS && error != 0) Serial.print(" -> U= "),Serial.print(SPG), Serial.println("V");
  return SPG; 
  
}

