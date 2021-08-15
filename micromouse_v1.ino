#define LED 2 // GPIO 2, build-in LED

void setup(){
    pinMode(LED, OUTPUT);
}

void loop(){
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
}

// git test