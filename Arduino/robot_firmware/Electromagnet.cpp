#include "Electromagnet.h"

Electromagnet::Electromagnet(int control_pin) {
  c_pin = control_pin;
  pinMode(c_pin, OUTPUT);
  digitalWrite(c_pin, LOW);
}


/* Set the power state of the Electromagnet */
void Electromagnet::setState(boolean state) {
  c_state = state;
  digitalWrite(c_pin, state);
  
  pf("Electromagnet set to: %s", c_state ? "ON" : "OFF");
}

/* Toggle function for the Electromagnet */
void Electromagnet::toggle() {
  setState(!c_state);
}
