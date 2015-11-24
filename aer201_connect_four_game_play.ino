// game play algorithm
// 1. keep reading the whiskers(1-7) into the game board to constantly update the game status
// 2. calculate the best move each time the opponent makes a move or we make a move (in calcNextMove()) and move the servo according to the calculated best move
// 3. after we make a move (as indicated by a whiskerFunnel, which we placed to detect our ball going into the game board) send a RF signal to the other robot to obtain the next ball

// pin assignments
const char whisker1 = 10;
const char whisker2 = 9;
const char whisker3 = 8;
const char whisker4 = 7;
const char whisker5 = 6;
const char whisker6 = 5;
const char whisker7 = 4;
const char whiskerFunnel = 2;
const char columnSelectServoPin = 3;
const char RF = 12;

#include <Servo.h>
Servo columnSelectServo;

byte gameStatus [8][9]=
{
{8,8,8,8,8,8,8,8,8}, // row "7"
{8,0,0,0,0,0,0,0,8}, // row 6
{8,0,0,0,0,0,0,0,8}, // row 5
{8,0,0,0,0,0,0,0,8}, // row 4
{8,0,0,0,0,0,0,0,8}, // row 3
{8,0,0,0,0,0,0,0,8}, // row 2
{8,0,0,0,0,0,0,0,8}, // row 1
{8,8,8,8,8,8,8,8,8} // row "0"
};

// 0 = no ball placed here
// 1 = our ball
// 2 = opp's ball (or should we use -1 instead?
// 8 = boundary

int lowestEmpty[9] = {8,1,1,1,1,1,1,1,8}; // 3rd # here = gameStatus[#th row][3rd col]
// an array that keeps track of the lowest empty spots / row to be considered for the next move in each column

// CONSTANTS that are to be used only for this calculation

char priorityList[5][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}}; // should we use dynamic allocation instead?
int listN = 0;
int targetCol = 1;
bool ourMove = false;

const byte EMPTY = 0;
const byte OURball = 1;
const byte OPPball = 2;
const byte BOUNDARY = 8;
const byte WHISKERdelay = 3000;

unsigned long ignoreServoWhiskerStart = 999999;
unsigned long ignoreColWhiskers = 999999;

void setup()
{
  Serial.begin(9600);
  pinMode(whisker1, INPUT);
  pinMode(whisker2, INPUT);
  pinMode(whisker3, INPUT);
  pinMode(whisker4, INPUT);
  pinMode(whisker5, INPUT);
  pinMode(whisker6, INPUT);
  pinMode(whisker7, INPUT);
  pinMode(whiskerBot, INPUT);
  pinMode(whiskerFunnel, INPUT);
  pinMode(RF, OUTPUT);
  pinMode(columnSelectServoPin, OUTPUT);

  columnSelectServo.attach(columnSelectServoPin);
}


void loop()
{
  Serial.println("--------------\nignoreColWhiskers = "+String(ignoreColWhiskers));
  Serial.println("now - ignoreColWhiskers = "+String(millis()-ignoreColWhiskers));
  bool refreshOurMove = readOurMove();
  Serial.println("OUR MOVE??? - "+String(refreshOurMove));
  if (!ourMove && refreshOurMove)
  {
    ourMove = true;
  }
  Serial.println("ourMove = "+String(ourMove));
  // update if previously not detected our move
  // don't update if there's an our move to be recorded

  if ((millis()-ignoreColWhiskers) < WHISKERdelay && !ourMove)
    // our move TRUE: could be our ball coming in right after opp's ball
  {
    return;
  }
  byte moveCol = readColWhiskers();
  Serial.println("-check-move in col "+String(moveCol));

  if (moveCol != 0)
  {
    ignoreColWhiskers = millis();
    gameStatus[lowestEmpty[moveCol]][moveCol] = OPPball*(!ourMove) + OURball*ourMove;
    if (ourMove)
    {
      ignoreServoWhiskerStart = millis();
      //ignoreColWhiskers = ignoreColWhiskers + WHISKERdelay*2/3; // in case OPP plays their ball right after we do
      Serial.println("*******OUR MOVE!!");
      delay(1000);
      ourMove = false;
    }
    Serial.println("-move detected-move in col "+String(moveCol)+" & row "+String(lowestEmpty[moveCol]) );
    Serial.println("-ball placed:"+String(gameStatus[lowestEmpty[moveCol]][moveCol]));
    lowestEmpty[moveCol] += 1;

  }

  calcNextMove();
  selectCol(); // move the servo to the angle for specific column &&&&&& send RF
  delay(1000);
}

void calcNextMove()
{
  int c4c = 0;

  const byte DIRECTIONincrements[7][2] = {{1,1},{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1}};
  byte rowIncrement;
  byte colIncrement;

  for (char colChecking = 1; colChecking <= 7; colChecking++)
  // up: 1, down: -1; right: 1, left: -1
  // [UP/DOWN, RIGHT/LEFT]
  //  1,  1 = up L;    NOT checking up;       1, 1 = up R;
  //  0, -1 = L;       TARGET SPOT;           0, 1 = R;
  // -1, -1 = down L;  -1, 0 = down;         -1, 1 = down R;
  {
    for (char direction = 0; direction < 7; direction++)
    {
      rowIncrement = DIRECTIONincrements[direction][0];
      colIncrement = DIRECTIONincrements[direction][1];
      checkOPP(colChecking, lowestEmpty[colChecking], rowIncrement, colIncrement);
      checkOUR(colChecking, lowestEmpty[colChecking], rowIncrement, colIncrement);
    }
  }
}

char readColWhiskers()
{
  char col1 = digitalRead(whisker1);
  char col2 = digitalRead(whisker2);
  Serial.println("col2*2 = "+String(col2*2));
  char col3 = digitalRead(whisker3);
  char col4 = digitalRead(whisker4);
  char col5 = digitalRead(whisker5);
  char col6 = digitalRead(whisker6);
  char col7 = digitalRead(whisker7);
  return col1*1 + col2*2 + col3*3 + col4*4 + col5*5 + col6*6 + col7* 7;
}

bool readOurMove()
{
  // record OUR move
  if ( (millis()-ignoreServoWhiskerStart) < WHISKERdelay)
  {
    return 0;
  }

  bool ourMove = digitalRead(whiskerFunnel);
  Serial.println("read our move: "+String(ourMove));
  ignoreServoWhiskerStart = millis()*ourMove + ignoreServoWhiskerStart*(!ourMove);
  return ourMove;
}

void selectCol()
{
  // select the column to place a ball for the next move
  byte angle;
  char currentHighestPriority;
  byte colSelected = 1;

  for (char counter = 0; counter < 5; counter++)
  {
    // compare if this spot on the priorityList is higher than a previous selected one
    if (lowestEmpty[priorityList[counter][1]] <= 6)
    {
      bool replace = priorityList[counter][0] >= currentHighestPriority;
    // if so, update it; if not, keep it
     currentHighestPriority = priorityList[counter][0]* replace + currentHighestPriority * (! replace);
     colSelected = priorityList[counter][1]*replace + colSelected*(!replace);
    }
  }

  sendRF();

  // calculate angle
  angle = map(colSelected, 1, 7, 20, 142);
  Serial.println("col selected: "+String(colSelected)+ " @ angle " + String(angle));
  Serial.println("will be place in row "+String(lowestEmpty[colSelected]));

  // turn the servo
  columnSelectServo.write(angle);
}



void checkOPP(char targetCol, char targetRow, byte rowIncrement, byte colIncrement)
{
  char ball = 0;
  for (char c4c = 0; ball != (OPPball || EMPTY); c4c++)
  {
    ball = gameStatus[targetRow + rowIncrement*c4c + rowIncrement][targetCol + colIncrement*c4c + colIncrement];
    if (ball == BOUNDARY)
    {
      return;
    }
    if (c4c >= 2)
    {
      priorityListManip(c4c+6, targetRow, targetCol); // priority 6, if compared success, put onto priorityList
    }
  }
  return;
  // exits for loop when it runs into our ball
}

void checkOUR(char targetCol, char targetRow, byte rowIncrement, byte colIncrement)
{
  char ball = 0;
  for (char c4c = 0; ball != (OURball || EMPTY); c4c++) //c4c = connect 4 count
  {
    ball = gameStatus[targetRow + rowIncrement*c4c + rowIncrement][targetCol + colIncrement*c4c + colIncrement];
    if (ball == BOUNDARY)
    {
      return;
    }
    if (c4c >= 2)
    {
      priorityListManip(c4c, targetRow, targetCol); // priority 6, if compared success, put onto priorityList
    }
  }
  return;

}


void priorityListManip(char priority, char targetRow, char targetCol)
{
  /*RETURN:
  1 if it's gonna be added to the priority list
  0 if ignore this spot*/
  // priorityList = [[priority,spot1],[priority,spot2],...]
  if ( priority >= priorityList[0][0] ) // priorityList[0] = highest priority
  {
    // insert the spot into the list
    for ( int n=4; n > 0; n--)
    {
      priorityList[n][0] = priorityList[n-1][0];
      priorityList[n][1] = priorityList[n-1][1];
      priorityList[n][2] = priorityList[n-1][2];
    }
    priorityList[0][0] = priority;
    priorityList[0][1] = targetRow;
    priorityList[0][2] = targetCol;
    return;
  }
  return;
}

void sendRF()
{
  // will implement when RF arrives
  return;
}
