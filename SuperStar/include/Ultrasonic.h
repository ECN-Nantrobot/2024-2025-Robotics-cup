#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <Arduino.h>

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
        // Prepare le signal
        digitalWrite(trig_pin, LOW);
        delayMicroseconds(2);
        // Créer une impulsion de 10 µs
        digitalWrite(trig_pin, HIGH);
        delayMicroseconds(TRIG_PULSE_DURATION_US);
        digitalWrite(trig_pin, LOW);

        // Renvoie le temps de propagation de l'onde (en µs)
        long ultrason_duration = pulseIn(echo_pin, HIGH);

        // Calcul de la distance en mètres
        return ultrason_duration * SOUND_SPEED/2 * 0.000001;
    }
};

#endif //ULTRASONIC_H
