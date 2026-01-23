#!/usr/bin/env python3
"""
Audio Engine für Sound-Wiedergabe.

WICHTIG: 
- Verwendet subprocess.Popen statt subprocess.call (nicht-blockierend)
- Audio-Device ist konfigurierbar über Environment-Variable AUDIO_DEVICE
- Default: plughw:2,0 (WM8960 HAT)
- Lautstärke-Steuerung via amixer (für Speech-Out Priorität)
"""

import os
import subprocess

# Absoluter Pfad zum Sound-Ordner
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SOUND_DIR = os.path.join(BASE_DIR, "sounds")

# Audio-Device ist konfigurierbar über Environment-Variable
# Default: WM8960 Sound HAT (per Kartenname für Stabilität)
AUDIO_DEVICE = os.environ.get("AUDIO_DEVICE", "plughw:CARD=wm8960soundcard,DEV=0")

# Soundkarten-Name für amixer (muss zur AUDIO_DEVICE passen)
AUDIO_CARD = os.environ.get("AUDIO_CARD", "wm8960soundcard")

# Lautstärke-Konfiguration
VOLUME_NORMAL = 100      # Normale Lautstärke (%)
VOLUME_SPEAKING = 25     # Reduzierte Lautstärke wenn Speech-Out spricht (%)

# Hält eine Referenz auf den aktuell laufenden Sound-Prozess
_current_sound_process = None

# Aktuelle Lautstärke (für Tracking)
_current_volume = VOLUME_NORMAL


def play_sound(filename, blocking=False):
    """
    Spielt eine WAV-Datei mit aplay.
    
    WICHTIG: Standardmäßig nicht-blockierend (Fire-and-Forget), damit
    die State Machine nicht während der Sound-Wiedergabe hängen bleibt.
    
    Args:
        filename: Dateiname der WAV-Datei (z.B. "greeting.wav")
        blocking: Wenn True, wartet bis der Sound fertig ist (default: False)
    
    Returns:
        bool: True wenn der Sound gestartet wurde, False bei Fehler
    """
    global _current_sound_process
    
    if not filename:
        return False
        
    path = os.path.join(SOUND_DIR, filename)
    
    if not os.path.exists(path):
        print(f"[AUDIO_ENGINE] Sound not found: {path}")
        return False

    print(f"[AUDIO_ENGINE] Playing: {path} (device: {AUDIO_DEVICE})")
    
    try:
        # Vorherigen Sound-Prozess beenden (falls noch läuft)
        stop_current_sound()
        
        if blocking:
            # Blockierender Modus (nur wenn explizit gewünscht)
            result = subprocess.call(["aplay", "-D", AUDIO_DEVICE, path])
            return result == 0
        else:
            # Nicht-blockierender Modus (Fire-and-Forget)
            # Der Sound läuft im Hintergrund weiter
            _current_sound_process = subprocess.Popen(
                ["aplay", "-D", AUDIO_DEVICE, path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        return True
        
    except FileNotFoundError:
        print("[AUDIO_ENGINE] ERROR: 'aplay' command not found. Is alsa-utils installed?")
        return False
    except Exception as e:
        print(f"[AUDIO_ENGINE] Error playing sound: {e}")
        return False


def stop_current_sound():
    """
    Stoppt den aktuell laufenden Sound (falls vorhanden).
    Nützlich um Sounds zu unterbrechen wenn ein neuer State kommt.
    """
    global _current_sound_process
    
    if _current_sound_process is not None:
        try:
            # Prüfen ob Prozess noch läuft
            if _current_sound_process.poll() is None:
                _current_sound_process.terminate()
                # Kurz warten auf saubere Beendigung
                try:
                    _current_sound_process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    _current_sound_process.kill()
                print("[AUDIO_ENGINE] Stopped current sound")
        except Exception as e:
            print(f"[AUDIO_ENGINE] Error stopping sound: {e}")
        finally:
            _current_sound_process = None


def play_state_sound(filename):
    """
    Spielt einen State-spezifischen Sound ab (nicht-blockierend).
    
    Args:
        filename: Dateiname der WAV-Datei (z.B. "greeting.wav")
    
    Returns:
        bool: True wenn der Sound gestartet wurde, False bei Fehler
    """
    if not filename:
        return False
    return play_sound(filename, blocking=False)


def is_sound_playing():
    """
    Prüft ob gerade ein Sound abgespielt wird.
    
    Returns:
        bool: True wenn Sound läuft, False sonst
    """
    global _current_sound_process
    
    if _current_sound_process is None:
        return False
    
    return _current_sound_process.poll() is None


def get_audio_device():
    """
    Gibt das aktuell konfigurierte Audio-Device zurück.
    
    Returns:
        str: Audio-Device String (z.B. "plughw:2,0")
    """
    return AUDIO_DEVICE


# ============================================================
# Lautstärke-Steuerung (für Speech-Out Priorität)
# ============================================================

def set_volume(percentage):
    """
    Setzt die Lautstärke der Soundkarte auf einen Prozentwert.
    
    Verwendet amixer um die System-Lautstärke zu ändern.
    Wichtig für Speech-Out Priorität: Wenn der Roboter spricht,
    sollen unsere Signal-Sounds leiser sein.
    
    Args:
        percentage: Lautstärke in Prozent (0-100)
        
    Returns:
        bool: True wenn erfolgreich, False bei Fehler
    """
    global _current_volume
    
    # Wert begrenzen
    percentage = max(0, min(100, percentage))
    
    print(f"[AUDIO_ENGINE] Setting volume to {percentage}%")
    
    try:
        # amixer mit der WM8960 Soundkarte verwenden
        # Versuche verschiedene Mixer-Controls (je nach Konfiguration)
        mixer_controls = ["Speaker", "Playback", "Master", "PCM"]
        
        success = False
        for control in mixer_controls:
            try:
                result = subprocess.run(
                    ["amixer", "-c", AUDIO_CARD, "sset", control, f"{percentage}%"],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode == 0:
                    print(f"[AUDIO_ENGINE] Volume set via '{control}' control")
                    success = True
                    break
            except Exception:
                continue
        
        if not success:
            # Fallback: Ohne Kartenangabe versuchen
            result = subprocess.run(
                ["amixer", "sset", "Master", f"{percentage}%"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                print(f"[AUDIO_ENGINE] Volume set via default Master control")
                success = True
        
        if success:
            _current_volume = percentage
            return True
        else:
            print(f"[AUDIO_ENGINE] WARNING: Could not set volume (no suitable mixer control found)")
            return False
            
    except FileNotFoundError:
        print("[AUDIO_ENGINE] ERROR: 'amixer' command not found. Is alsa-utils installed?")
        return False
    except subprocess.TimeoutExpired:
        print("[AUDIO_ENGINE] ERROR: amixer timeout")
        return False
    except Exception as e:
        print(f"[AUDIO_ENGINE] Error setting volume: {e}")
        return False


def set_volume_for_speaking(is_speaking):
    """
    Passt die Lautstärke an, je nachdem ob Speech-Out gerade spricht.
    
    Wenn Speech-Out spricht, werden unsere Sounds leiser (25%),
    damit die Sprachausgabe gut verständlich bleibt.
    Wenn Speech-Out fertig ist, wird die Lautstärke wieder auf 100% gesetzt.
    
    Args:
        is_speaking: True wenn Speech-Out gerade spricht, False sonst
        
    Returns:
        bool: True wenn erfolgreich, False bei Fehler
    """
    if is_speaking:
        print("[AUDIO_ENGINE] Speech-Out active -> reducing volume to speaking level")
        return set_volume(VOLUME_SPEAKING)
    else:
        print("[AUDIO_ENGINE] Speech-Out finished -> restoring normal volume")
        return set_volume(VOLUME_NORMAL)


def get_current_volume():
    """
    Gibt die aktuell gesetzte Lautstärke zurück.
    
    Returns:
        int: Aktuelle Lautstärke in Prozent
    """
    return _current_volume


def restore_normal_volume():
    """
    Stellt die normale Lautstärke (100%) wieder her.
    
    Returns:
        bool: True wenn erfolgreich, False bei Fehler
    """
    return set_volume(VOLUME_NORMAL)
