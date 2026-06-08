#include "display.h"

Display::Display()
: _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) // this init for class has to be here not in the curly brackets!!!
{
}

void Display::init(){
  if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
  }
  _display.clearDisplay();

  _display.setTextSize(1);      // Normal 1:1 pixel scale
  _display.setTextColor(SSD1306_WHITE); // Draw white text
  _display.setCursor(0, 0);     // Start at top-left corner
  _display.cp437(true);         // Use full 256 char 'Code Page 437' font

  _display.printf("Robot controller\n%s\n", VERSION);

  _display.display();
}

void Display::displayInfo(float q[JOINT_NUM], float x[6]){
  constexpr int q_start_x = 0;
  constexpr int q_start_y = 8;
  constexpr int x_pos_start_x = 67;
  constexpr int x_pos_start_y = 8;
  constexpr int x_ang_start_x = x_pos_start_x + 30;
  constexpr int x_ang_start_y = 8;

  _display.clearDisplay();

  _display.setTextSize(1);
  _display.setTextColor(SSD1306_WHITE);
  _display.setCursor(0, 0);
  _display.print("q [rad]  x [mm] [rad]");

  for (int i=0; i<JOINT_NUM; i++){
    _display.setCursor(q_start_x, q_start_y * (i+1));
    _display.printf("%.2f", q[i]);
  }

  for (int i=0; i<3; i++){
    _display.setCursor(x_pos_start_x, x_pos_start_y * (i+1));
    _display.printf("%.2f", x[i]); 

    _display.setCursor(x_ang_start_x, x_ang_start_y * (i+1));
    _display.printf("%.2f", x[i+3]);
  }
  _display.display();
}
