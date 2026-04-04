int pin_electrovanne = 12;
int pin_pompe = 13;

void lacher() {
  digitalWrite(pin_pompe, LOW);
  digitalWrite(pin_electrovanne, HIGH);
}

void accrocher() {
  digitalWrite(pin_pompe, HIGH);
  digitalWrite(pin_electrovanne, LOW);
}

void setup() {
  pinMode(pin_pompe, OUTPUT);
  pinMode(pin_electrovanne, OUTPUT);
}

void loop() {
  lacher();
  delay(1000);
  accrocher();
  delay(2000);
}
