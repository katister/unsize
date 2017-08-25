/* Base code found at https://www.sparkfun.com/products/9117
 * Modified to be able to count number of revolutions of the encoder
 * 
 * JT Hinchen   2.2.17
 */


#define pinButton 3
#define pinA 4
#define pinB 5

int count;
int Rotations;
int resolutionx2=15;      //using 2x the resolution in degrees to keep data in int type
int Angle;
float inchpoly,inchpwr,inchlin,inchround;

void setup() {
  ConfigEncoder();
  Serial.begin(115200); 
  Serial.println("start");
}

void loop() {
  int Position, Press;
  int Direction=0;
  ConfigEncoder();
  ReadEncoder(Position, Press);
  while (!Serial.available()){
    int Position2, Press2;
    do{
      ReadEncoder(Position2, Press2);
    } while((Position2==Position)&&(Press2==Press));
    if(Position2!=Position){
      int Direction=((Position == 0) && (Position2 == 1)) ||
        ((Position == 1) && (Position2 == 3)) ||
        ((Position == 3) && (Position2 == 2)) ||
        ((Position == 2) && (Position2 == 0));         
      if(Direction==1){
        count++;
      }      
     else{
        count-- ;
      }
      /*Display rotation count:
       * Rotations=count/48;                       //48 counts per revolution
      Angle=(count-48*Rotations)*resolutionx2/2;*/

      //Display 
      inchpoly=-0.00002*pow(count,2)+.1571*count-.0947;
      inchpwr=.1525*pow(count,1.0007);
      inchlin=.1516*count+.1965;

      inch
      
      Serial.print(inchpoly);                  //print number of counts
      Serial.print(" ");   
      Serial.print(inchpwr);   
      Serial.print(" ");   
      Serial.println(inchlin);   
    }
    
    if (Press2 != Press)
    {
      Serial.println(Press ? "Press" : "Release - Reset");    //reset the counter when button is pressed
      count=0;
    }
    
    Position = Position2;
    Press = Press2;
  }
}

void ConfigEncoder(){             //configure pins
  pinMode(pinA,INPUT);
  pinMode(pinB,INPUT);
  pinMode(pinButton,INPUT);
  digitalWrite(pinA,HIGH);
  digitalWrite(pinB,HIGH);
  digitalWrite(pinButton,HIGH);
}

void ReadEncoder(int &rotate, int& buttonPress){    //register encoder state
  rotate=2*digitalRead(pinB)+digitalRead(pinA);
  buttonPress=digitalRead(pinButton);
}

