/*
  May the robot rescue you.
*/

// --------------------------   DEFINITONS   ----------------------------------
// :: DEBUG ::
#define MY_DEBUG 0     // set this to 1, to activate debug print out to the sm.

// :: ENGINE ::
// Pin layout
#define P_ENG_LEFT_DIR 13
#define P_ENG_RIGH_DIR 12
#define P_ENG_LEFT_SPD 11
#define P_ENG_RIGH_SPD 3
#define P_ENG_LEFT_BRK 8
#define P_ENG_RIGH_BRK 9

// direction
#define ENG_DIR_FORW HIGH
#define ENG_DIR_BACK LOW

// break
#define ENG_BREAK_ON HIGH
#define ENG_BREAK_OFF LOW

// speed values for turning from left to right
// the amount of values is equivalent to the
// amount of reflection sensors
const int ENG_SPD_DIR[5][2] = { {-127, 127}, {0, 127}, {127, 127}, {127, 0}, {127, -127} };

// speed multiplicator for the ENG_SPD_DIR
#define SPD_DEF 0.4f
#define SPD_MAX 2.0f
#define SPD_OFF 0.0f

// :: REFLECTION SENSORS ::
// Pin layout from left to right.
const int P_RS_VL[] = { A8, A9, A10, A11, A12 };

// Tresholds for each sensor
const int RS_TH[] = { 650, 650, 650, 650, 650 };

// Priorites of the sensors in ``normal`` mode.
const int RS_PRIO[] = { 2,   1,   3,   0,   4 };


// --------------------------   GLOBALS   -------------------------------------
int state = 0;                           //                   the current state
float cur_speed = 1;                     //                   the current speed
byte cur_spd_dir = 255;                  //             the current break state
int sv_analog[] = {0, 0, 0, 0, 0};       //               sensor values: analog
boolean sv[] = {0, 0, 0, 0, 0};          // sensor values:   boolean (is black)
boolean sv_prev[] = {0, 0, 0, 0, 0};     // sensor values:     boolean previous
boolean sv_pprev[] = {0, 0, 0, 0, 0};    // sensor values:    boolean prev prev
boolean sv_lworking[] = {0, 0, 0, 0, 0}; // sensor values: boolean last working


// some loop indices
int i, index;


// --------------------------   ROUTINES   ------------------------------------
/*
  The setup routine should initialize all pins.
*/
void setup() {
  // initialize pins
  pinMode(P_ENG_LEFT_DIR, OUTPUT);
  pinMode(P_ENG_RIGH_DIR, OUTPUT);
  pinMode(P_ENG_LEFT_SPD, OUTPUT);
  pinMode(P_ENG_RIGH_SPD, OUTPUT);
  pinMode(P_ENG_LEFT_BRK, OUTPUT);
  pinMode(P_ENG_RIGH_BRK, OUTPUT);

#if MY_DEBUG == 1
  Serial.begin(9600);  // DEBUG: Starts the serial monitor.
#endif
}

/*
  Reads the value of all sensors and stores this
  value to the sv array as booleans - whether
  value is over threshold.
*/
void read_sensors() {
  for (i=0; i<5; i++){
    // Reads the analog values.
    sv_analog[i] = analogRead(P_RS_VL[i]);
    // Saves previous states.
    sv_pprev[i] = sv_prev[i];
    sv_prev[i] = sv[i];
    // Saves the current state.
    sv[i] = (sv_analog[i] >= RS_TH[i]) ? true : false;
  }
}

#if MY_DEBUG == 1
/* DEBUG: print out all senser values */
void write_sensors() {
  for (i=0; i<5; i++){
    Serial.print(i); Serial.print(": ");
    Serial.print(sv_analog[i]);
    Serial.print(" {"); Serial.print(sv[i]); Serial.print("}\t");
  }
  Serial.print("\n");
}
#endif

/*
  Sets the engines into the correct state for the
  given direction index. Once called with a cur_speed > 0
  this we enable the engines to drive. Therefore this
  function could only be called on state changes.
*/
void drive(byte spd_dir_index, float speed){
  // If this function is called with same parameters twice
  // one does not need to change the values.
  if (cur_spd_dir == spd_dir_index && cur_speed == speed){
    return;
  }

  // Get engine speed values for the given index.
  const int* dir = ENG_SPD_DIR[spd_dir_index];
  int left = dir[0] * speed;
  int right = dir[1] * speed;

  // If speed less then zero than break.
  if (speed <= 0) {
    digitalWrite(P_ENG_LEFT_BRK, ENG_BREAK_ON);
    digitalWrite(P_ENG_RIGH_BRK, ENG_BREAK_ON);
  } else {
    digitalWrite(P_ENG_LEFT_BRK, ENG_BREAK_OFF);
    digitalWrite(P_ENG_RIGH_BRK, ENG_BREAK_OFF);
  }

  // The left and right values now may be in the range [-255, 255],
  // which will not directly work for the analog write. Therefore
  // we take the absolute values for the speed, but if the given speed was < 0
  // we will switch the direction to backwards.
  analogWrite(P_ENG_LEFT_SPD, abs(left));
  digitalWrite(P_ENG_LEFT_DIR, (left >= 0) ? ENG_DIR_FORW : ENG_DIR_BACK );

  analogWrite(P_ENG_RIGH_SPD, abs(right));
  digitalWrite(P_ENG_RIGH_DIR, (right >= 0) ? ENG_DIR_FORW : ENG_DIR_BACK);

  cur_spd_dir = spd_dir_index;
  cur_speed = speed;
}

/*
  Checks whether all values in sv are true.
*/
boolean all_black(){
  boolean black = true;
  for (i=0; i<5; i++){
    black = black && sv[i];
  }
  return black;
}

/*
  Checks if at least one value of sv is true.
*/
boolean one_black(){
  for (i=0; i<5; i++){
    if (sv[i]) {
      return true;
    }
  }
  return false;
}

/*
  STATE:0
  Constantly drives forward until at least one of the
  sensors registers a black value.
*/
void find_line_first(){
  // set dircetion to straight forward
  drive(2, SPD_DEF);
  if (one_black()){
    state = 1;
  }
}

/*
  STATE:1
  Tries to drive to the direction where a sensor notices a black value.
  It checks from the inner sensors to the outer ones (defined in the
  RS_PRIO array).
  If no sensor reacts, the state machine goes back to STATE:0 .
*/
void follow_mid_sensor(){
  for (i=0; i<5; i++){
    index = RS_PRIO[i];
    if (sv[index]) {
      drive(index, SPD_DEF);
      return;
    }
  }
// If non of the RS values were true the robot did not see a line.
// Now we check how the previous sensor values were set.
// And switch therefore into the next state :)
  state = 2;
}

/*
  STATE:2
  This should check the last working sensor values and drive
  into that directions that found a black vlaue. Here we switch
  the priorities so that the outter sernsor values have a higher
  priority than the middle sensor.
*/
void check_last_working(){
//  First check whether we are back on track
  if (one_black()) {
    state = 1;
    return;
  }

//  Otherwise head into the last working direction
//  with vice-versa priorities.
  for (i=4; i>=0; i--){
    index = RS_PRIO[i];
    if (sv_lworking[index]) {
      drive(index, SPD_DEF);
      return;
    }
  }

// If none of the last_working values was set
// go back to the first state.
  state = 0;
}

/*
  STATE:3
  Stops the robot and changes state to follow mid sensor.
*/
void stop_bot(){
  drive(2, SPD_OFF);
  state = 1;
}

/*
  The loop routine is called by the arduino all the time.
  This will represent our state machine.
*/
void loop() {
  read_sensors();

  // if all sensors are black we should stop
  // if at least one sensor sees black, save
  // this configuration as last working configuration
  if (all_black()){
    state = 3;
  } else if (one_black()){
    for (i=0; i<5; i++){
      sv_lworking[i] = sv[i];
    }
  }

#if MY_DEBUG == 1
  write_sensors(); // DEBUG: prints all sensor values to the serial monitor
#endif

  switch (state) {
    case 0:
      find_line_first();
      break;
    case 1:
      follow_mid_sensor();
      break;
    case 2:
      check_last_working();
      break;
    case 3:
      stop_bot();
      break;
    default:
      find_line_first();
      break;
  }
}
