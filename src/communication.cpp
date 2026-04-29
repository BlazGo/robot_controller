#include "communication.h"

ComHandler::ComHandler(HardwareSerial &s)
  : stream(s) { }

void ComHandler::begin(unsigned long baud) {
    stream.begin(baud);
}

void ComHandler::update() {
  while (stream.available()){
    char latest_char = stream.read();

    if (latest_char == START_CHAR){
      ndx = 0;
      receiving = true;
      continue;
    }

    if (latest_char == END_CHAR){
      receivedChars[ndx] = '\0';
      receiving = false;

      stream.println(F("Message received:"));
      stream.println(receivedChars);

      if (parseMessage()){
        stream.println(F("Parsing OK"));
        cmdReady = true;
      }
      else{
        stream.println(F("Parsing FAILED"));
      }
      continue;
    }

    // Store character safely
    if (ndx < MAX_MSG_LEN - 1){
      receivedChars[ndx++] = latest_char;
    }
    else{
      // Overflow protection
      receiving = false;
      ndx = 0;
      stream.println(F("Message too long"));
    }
  }
}

bool ComHandler::parseMessage(){
  char tempChars[MAX_MSG_LEN];      // temporary array used for parsing

  strcpy(tempChars, receivedChars); // Copy into new variable to not destroy OG

  char *token = strtok(tempChars, SEPARATOR); // get the up to separator
  // check if empty
  if (token == NULL) 
    return false;
    
  const CommandInfo* cmdInfo = nullptr;

  // Find matching commant
  for (uint8_t i = 0; i < NUM_COMMANDS; i++){
    if (strcmp(token, command_table[i].name) == 0){
      cmdInfo = &command_table[i];
      break;
    }
  }
  
  // Check if a command has been found
  if (cmdInfo == nullptr){
    return false;
  }

  cmd_robot.type = cmdInfo->type;
  cmd_robot.param_count = 0;

  // Parsing parameters
  while ((token = strtok(NULL, SEPARATOR)) != NULL){
    if (cmd_robot.param_count >= MAX_PARAMS){
      return false;
    }
    // Convert to double precision float and save
    cmd_robot.params[cmd_robot.param_count++] = atof(token);
  }
  // Check number of params
  if (cmd_robot.param_count != cmdInfo->param_count){
    return false;
  }
  return true;
}