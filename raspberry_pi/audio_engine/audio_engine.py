import pygame
import time

pygame.mixer.init()

def play_sound(name):
    try:
        pygame.mixer.music.load(f"sounds/{name}.wav")
        pygame.mixer.music.play()
    except:
        print("Sound missing:", name)

