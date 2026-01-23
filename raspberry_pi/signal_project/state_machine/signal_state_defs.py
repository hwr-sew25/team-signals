from enum import Enum


class Priority(Enum):
    """
    Prioritätsstufen für Signal-States.
    
    P0 (Must / Safety & Demo-stopper): Sicherheitskritisch, verhindert Schäden,
       oder ohne das ist die Demo "kaputt".
    P1 (Core Demo Flow): Kern-User-Journey im Foyer (Begrüßung, Sprechen, 
       Bewegung, Ziel erreicht).
    P2 (Nice-to-have / UX-Polish): Verbessert Verständlichkeit, aber Demo 
       funktioniert auch ohne.
    P3 (Backlog / später): Unklar definiert oder geringe Wirkung im Demo-Kontext.
    """
    P0 = 0  # Must / Safety & Demo-stopper
    P1 = 1  # Core Demo Flow
    P2 = 2  # Nice-to-have / UX-Polish
    P3 = 3  # Backlog / später


class SignalState(Enum):
    GREETING = 0
    IDLE = 1
    BUSY = 2
    STOP_BUSY = 3
    ERROR_MINOR_STUCK = 4
    ERROR_MINOR_NAV = 5
    ROOM_NOT_FOUND = 6
    ERROR_MAJOR = 7
    LOW_BATTERY = 8
    MOVE_LEFT = 9
    MOVE_FORWARD = 10
    MOVE_RIGHT = 11
    MOVE_BACKWARD = 12
    START_MOVE = 13
    STOP_MOVE = 14
    GOAL_REACHED = 15
    REVERSE = 16
    SPEAKING = 17
    WAITING = 18


# Prioritäten-Mapping gemäß USE-CASE MATRIX
STATE_PRIORITY = {
    # P0 - Must / Safety & Demo-stopper
    SignalState.ERROR_MINOR_STUCK: Priority.P0,
    SignalState.ERROR_MINOR_NAV: Priority.P0,
    SignalState.ROOM_NOT_FOUND: Priority.P0,
    SignalState.ERROR_MAJOR: Priority.P0,
    SignalState.LOW_BATTERY: Priority.P0,
    SignalState.STOP_MOVE: Priority.P0,
    
    # P1 - Core Demo Flow
    SignalState.GREETING: Priority.P1,
    SignalState.MOVE_LEFT: Priority.P1,
    SignalState.MOVE_FORWARD: Priority.P1,
    SignalState.MOVE_RIGHT: Priority.P1,
    SignalState.MOVE_BACKWARD: Priority.P1,
    SignalState.START_MOVE: Priority.P1,
    SignalState.GOAL_REACHED: Priority.P1,
    SignalState.SPEAKING: Priority.P1,
    SignalState.WAITING: Priority.P1,  # Aktives Warten auf User-Input
    
    # P2 - Nice-to-have / UX-Polish
    SignalState.IDLE: Priority.P2,
    SignalState.BUSY: Priority.P2,
    SignalState.REVERSE: Priority.P2,
    
    # P3 - Backlog / später
    SignalState.STOP_BUSY: Priority.P3,
}


def get_priority(state: SignalState) -> Priority:
    """
    Gibt die Priorität eines States zurück.
    
    Args:
        state: Der SignalState
        
    Returns:
        Priority: Die Priorität des States (Default: P3)
    """
    return STATE_PRIORITY.get(state, Priority.P3)


def get_states_by_priority(priority: Priority) -> list:
    """
    Gibt alle States mit einer bestimmten Priorität zurück.
    
    Args:
        priority: Die gewünschte Priorität
        
    Returns:
        list: Liste von SignalStates mit dieser Priorität
    """
    return [state for state, prio in STATE_PRIORITY.items() if prio == priority]

