// Vitesse du son dans l'air
#define SOUND_SPEED 340
#define TRIG_PULSE_DURATION_US 10
struct capteur {
  int trig_pin;
  int echo_pin;

  capteur(int t_pin, int e_pin) : trig_pin(t_pin), echo_pin(e_pin) {}
  void setup() {
    pinMode(trig_pin, OUTPUT); // On configure le trig en output
    pinMode(echo_pin, INPUT); // On configure l'echo en input
  }
  float distance() {
    long ultrason_duration;

    // Prepare le signal
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    // Créer une impulsion de 10 µs
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(TRIG_PULSE_DURATION_US);
    digitalWrite(trig_pin, LOW);

    // Renvoie le temps de propagation de l'onde (en µs)
    ultrason_duration = pulseIn(echo_pin, HIGH);

    // Calcul de la distance
    return ultrason_duration * SOUND_SPEED/2 * 0.0001;
  }
};

capteur capteur1(13, 12);
capteur capteur2(14, 27);

float distance_cm;

void setup() {
  Serial.begin(115200);
  capteur1.setup();
  capteur2.setup();
}

float distance;

void loop() {
  // On calcule la distance 1
  distance = capteur1.distance();
  // On affiche la distance sur le port série
  Serial.print("Distance 1 (cm): ");
  Serial.println(distance);
  delay(1000);

  // On calcule la distance 2
  distance = capteur2.distance();
  // On affiche la distance sur le port série
  Serial.print("Distance 2 (cm): ");
  Serial.println(distance);

  delay(1000);
}

