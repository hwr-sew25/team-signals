#ifndef STATE_DEFS_H
#define STATE_DEFS_H

// ============================================================
// Signal States - Enum Definition
// ============================================================

enum SignalState {
    GREETING = 0,
    IDLE = 1,
    BUSY = 2,
    STOP_BUSY = 3,
    ERROR_MINOR_STUCK = 4,
    ERROR_MINOR_NAV = 5,
    ROOM_NOT_FOUND = 6,
    ERROR_MAJOR = 7,
    LOW_BATTERY = 8,
    MOVE_LEFT = 9,
    MOVE_FORWARD = 10,
    MOVE_RIGHT = 11,
    MOVE_BACKWARD = 12,
    START_MOVE = 13,
    STOP_MOVE = 14,
    GOAL_REACHED = 15,
    SPEAKING = 16,
    WAITING = 17,
    
    // Interner State für ungültige Zustände
    STATE_INVALID = 255
};

// ============================================================
// Prioritäten - Konsistent mit Python signal_state_defs.py
// ============================================================
// Niedrigerer Wert = HÖHERE Priorität!
// P0: Must / Safety & Demo-stopper (höchste)
// P1: Core Demo Flow
// P2: Nice-to-have / UX-Polish
// P3: Backlog / später (niedrigste)

#define PRIORITY_P0  0   // Höchste - Safety & Demo-stopper
#define PRIORITY_P1  1   // Core Demo Flow
#define PRIORITY_P2  2   // Nice-to-have / UX-Polish
#define PRIORITY_P3  3   // Niedrigste - Backlog/Default

// ============================================================
// Prioritäts-Mapping für jeden State
// ============================================================

inline uint8_t getStatePriority(SignalState state) {
    switch (state) {
        // P0 - Must / Safety & Demo-stopper (HÖCHSTE PRIORITÄT)
        case ERROR_MAJOR:       return PRIORITY_P0;
        case ERROR_MINOR_STUCK: return PRIORITY_P0;
        case ERROR_MINOR_NAV:   return PRIORITY_P0;
        case ROOM_NOT_FOUND:    return PRIORITY_P0;
        case LOW_BATTERY:       return PRIORITY_P0;
        case STOP_MOVE:         return PRIORITY_P0;
        
        // P1 - Core Demo Flow
        case GREETING:          return PRIORITY_P1;
        case MOVE_LEFT:         return PRIORITY_P1;
        case MOVE_FORWARD:      return PRIORITY_P1;
        case MOVE_RIGHT:        return PRIORITY_P1;
        case MOVE_BACKWARD:     return PRIORITY_P1;
        case START_MOVE:        return PRIORITY_P1;
        case GOAL_REACHED:      return PRIORITY_P1;
        case SPEAKING:          return PRIORITY_P1;
        case WAITING:           return PRIORITY_P1;
        
        // P2 - Nice-to-have / UX-Polish
        case IDLE:              return PRIORITY_P2;
        case BUSY:              return PRIORITY_P2;
        case STOP_BUSY:         return PRIORITY_P2;
        
        // Default: Niedrigste Priorität
        default:                return PRIORITY_P3;
    }
}

// ============================================================
// Prioritäts-Vergleichslogik
// ============================================================

// Prüft ob newState den currentState überschreiben darf
// Returns: true wenn newState aktiviert werden darf
inline bool canOverrideState(SignalState currentState, SignalState newState) {
    uint8_t currentPrio = getStatePriority(currentState);
    uint8_t newPrio = getStatePriority(newState);
    
    // Niedrigerer Prioritätswert = höhere Priorität
    // Gleiche oder höhere Priorität darf überschreiben
    // (newPrio <= currentPrio bedeutet: neuer State ist wichtiger oder gleich wichtig)
    return (newPrio <= currentPrio);
}

// ============================================================
// Spezial-States die immer überschreiben dürfen
// ============================================================

// Einige States sollten IMMER gesetzt werden können:
// - IDLE: Universeller Reset-State
// - STOP_MOVE: Sicherheitsrelevant - Bremsen immer erlauben
// - ERROR_MAJOR: Kritischer Fehler immer anzeigen
inline bool isForceAllowedState(SignalState state) {
    switch (state) {
        case ERROR_MAJOR:   return true;  // Kritische Fehler immer
        case STOP_MOVE:     return true;  // Sicherheitsstop immer
        default:            return false;
    }
}

// ============================================================
// State-Namen für Debug-Ausgabe (optional)
// ============================================================

#ifdef DEBUG_STATE_NAMES
inline const char* getStateName(SignalState state) {
    switch (state) {
        case GREETING:          return "GREETING";
        case IDLE:              return "IDLE";
        case BUSY:              return "BUSY";
        case STOP_BUSY:         return "STOP_BUSY";
        case ERROR_MINOR_STUCK: return "ERROR_MINOR_STUCK";
        case ERROR_MINOR_NAV:   return "ERROR_MINOR_NAV";
        case ROOM_NOT_FOUND:    return "ROOM_NOT_FOUND";
        case ERROR_MAJOR:       return "ERROR_MAJOR";
        case LOW_BATTERY:       return "LOW_BATTERY";
        case MOVE_LEFT:         return "MOVE_LEFT";
        case MOVE_FORWARD:      return "MOVE_FORWARD";
        case MOVE_RIGHT:        return "MOVE_RIGHT";
        case MOVE_BACKWARD:     return "MOVE_BACKWARD";
        case START_MOVE:        return "START_MOVE";
        case STOP_MOVE:         return "STOP_MOVE";
        case GOAL_REACHED:      return "GOAL_REACHED";
        case SPEAKING:          return "SPEAKING";
        case WAITING:           return "WAITING";
        default:                return "UNKNOWN";
    }
}
#endif

#endif
