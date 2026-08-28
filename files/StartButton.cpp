#include "StartButton.h"

volatile bool StartButton::pinChanged = false;

void StartButton::isrHandler() {
    // ISR curta: apenas sinaliza. Toda a logica pesada fica no update().
    pinChanged = true;
}

void StartButton::begin() {
    pinMode(PIN_START_BUTTON, INPUT_PULLUP);
    // Botao conectado entre o pino e GND: HIGH = solto, LOW = pressionado
    // (por causa do INPUT_PULLUP).

    attachInterrupt(digitalPinToInterrupt(PIN_START_BUTTON), isrHandler, CHANGE);

    isPressed = false;
    holdEventFired = false;
    pressStartMillis = 0;
    lastDebounceMillis = 0;
    lastRawState = HIGH; // estado solto
}

ButtonEvent StartButton::update() {
    bool rawState = digitalRead(PIN_START_BUTTON);
    unsigned long nowMillis = millis();

    // --- Debounce ---
    if (rawState != lastRawState) {
        lastDebounceMillis = nowMillis;
        lastRawState = rawState;
    }

    bool stableState = rawState;
    if ((nowMillis - lastDebounceMillis) < BUTTON_DEBOUNCE_MS) {
        // Ainda dentro da janela de debounce -- usa o ultimo estado
        // considerado estavel (nao o rawState mais recente).
        stableState = isPressed ? LOW : HIGH;
    }

    bool currentlyPressed = (stableState == LOW);

    // --- Transicao: solto -> pressionado ---
    if (currentlyPressed && !isPressed) {
        isPressed = true;
        pressStartMillis = nowMillis;
        holdEventFired = false;
    }

    // --- Transicao: pressionado -> solto ---
    if (!currentlyPressed && isPressed) {
        isPressed = false;
        holdEventFired = false;
    }

    // --- Enquanto pressionado, verifica se atingiu o tempo de hold ---
    if (isPressed && !holdEventFired) {
        if ((nowMillis - pressStartMillis) >= BUTTON_HOLD_TIME_MS) {
            holdEventFired = true; // garante disparo unico por pressao
            return BUTTON_HOLD_CONFIRMED;
        }
    }

    return BUTTON_NONE;
}
