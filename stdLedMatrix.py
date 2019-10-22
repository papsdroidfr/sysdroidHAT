#!/usr/bin/env python3
########################################################################
# Filename    : stdLedMatrix.py
# Description : affichage binaire sur une matrice de leds standards
# auther      : papsdroid.fr
# modification: 2019/10/01
########################################################################
import RPi.GPIO as GPIO
import time, threading

#classe pour gérer l'affichage en tâche de fond sur la matrice de leds
#-----------------------------------------------------------------------------------------
class LedMatrix(threading.Thread):
    def __init__(self, speedscroll=0.1, inverse=False):
        threading.Thread.__init__(self)  # appel au constructeur de la classe mère Thread
        self.etat=False             # True: affichage en cours, False: toutes leds éteintes.
        self.dataPin      = 11      # DS Pin 74HC595(Pin14)
        self.latchPin     = 13      # ST_CP Pin 74HC595(Pin12)
        self.clockPin     = 15      # SH_CP Pin 74HC595(Pin11)
        self.buffer8b=[0,0,0,0,0,0,0,0]         # 8 codes binaires 8bits pour préparer l'affichage
        self.buffer8bFinal=[0,0,0,0,0,0,0,0]    # 8 codes binaires 8bits affichage en cours
        self.afficheMsg = False          # True: affichage d'un message en cours.
        self.speedscroll = speedscroll   # vitesse de défilement des messages en secondes
        self.inverse = inverse           # inverse l'affichafe si True (miroir 180°)
        
        #pin associés à la matrice de leds
        GPIO.setup(self.dataPin, GPIO.OUT)
        GPIO.setup(self.latchPin, GPIO.OUT)
        GPIO.setup(self.clockPin, GPIO.OUT)

        self.puis2 = [1, 0b10, 0b100, 0b1000, 0b10000, 0b100000, 0b1000000, 0b10000000]
                      
        #dictionnaire des codes binaires représentant chaque charactère au format 5x3
        #chaque code binaire 0bxxxxx représente 5 LEds verticales à allumer/éteindre du haut vers le bas
        #0b100000: led du haut allumée 0b00001: led du bas allumée
        #une font 5*3 est ainsi représente par 3 codes binaires de 5bits
        self.font5x3 = {    
            "0" : [0b11111, 0b10001, 0b11111], # "0"
            "1" : [0b01001, 0b11111, 0b00001], # "1"
            "2" : [0b10011, 0b10101, 0b01001], # "2"
            "3" : [0b10101, 0b10101, 0b11111], # "3"
            "4" : [0b00110, 0b01010, 0b11111], # "4"
            "5" : [0b11101, 0b10101, 0b10111], # "5"
            "6" : [0b11111, 0b10101, 0b10111], # "6"
            "7" : [0b10001, 0b10110, 0b11000], # "7"
            "8" : [0b11111, 0b10101, 0b11111], # "8"
            "9" : [0b11101, 0b10101, 0b11111], # "9"
     
            "A" : [0b01111, 0b10100, 0b01111], # "A"
            "B" : [0b11111, 0b10101, 0b01010], # "B"
            "C" : [0b01110, 0b10001, 0b10001], # "C"
            "D" : [0b11111, 0b10001, 0b01110], # "D"
            "E" : [0b11111, 0b10101, 0b10101], # "E"
            "F" : [0b01111, 0b10100, 0b10100], # "F"
            "G" : [0b01110, 0b10001, 0b10111], # "G"
            "H" : [0b11111, 0b00100, 0b11111], # "H"
            "I" : [0b00000, 0b10111, 0b00000], # "I"
            "J" : [0b00011, 0b00001, 0b11110], # "J"
            "K" : [0b11111, 0b00100, 0b11011], # "K"
            "L" : [0b11110, 0b00001, 0b00001], # "L"
            "M" : [0b11111, 0b01000, 0b11111], # "M"
            "N" : [0b11111, 0b10000, 0b01111], # "N"
            "O" : [0b01110, 0b10001, 0b01110], # "O"
            "P" : [0b11111, 0b10100, 0b01100], # "P"
            "Q" : [0b01110, 0b10010, 0b01101], # "Q"
            "R" : [0b11111, 0b10110, 0b01101], # "R"
            "S" : [0b01001, 0b10101, 0b10011], # "S"
            "T" : [0b10000, 0b11111, 0b10000], # "T"
            "U" : [0b11110, 0b00001, 0b11110], # "U"
            "V" : [0b11100, 0b00011, 0b11100], # "V"
            "W" : [0b11111, 0b00100, 0b11111], # "W"
            "X" : [0b11011, 0b00100, 0b11011], # "X"
            "Y" : [0b11000, 0b00111, 0b11000], # "Y"
            "Z" : [0b10011, 0b10101, 0b11001], # "Z"
            "." : [0b00001],                   # "."
            "," : [0b00001, 0b00010],          # ","
            ":" : [0b01010],                   # ":"
            "-" : [0b00100, 0b00100],          # "-"
            "°" : [0b01000, 0b10100, 0b01000], # "°"
            "*" : [0b01010, 0b00100, 0b01010], # "*"
            "%" : [0b10011, 0b00100, 0b11001], # "%"
            " " : [ 0, 0, 0]                   # " "
        }


        #démarrage du thread
        self.start()

    #exécution du thread
    #-------------------
    def run(self):
        self.etat=True
        while (self.etat):
            self.buffer8bFinal = self.buffer8b
            self.affiche()   # affichage buffer sur la matrice de leds
            time.sleep(0.01) # mise en pause (sinon proc saturé)

    #arrêt du thread
    #---------------
    def stop(self):
        self.scrollmsg(' BYE ')
        self.etat=False
            
    #etteint la matrice
    #----------------------------------
    def clear(self):
        self.buffer8b = [0,0,0,0,0,0,0,0] ;
    
    #génère la liste des codes binaires depuis font5x3 à partir d'un texte
    #----------------------------------------------------------------------
    def create_msg(self, text):
        matrix= []
        for i in range(len(text)):
            if text[i].upper() in self.font5x3: # recherche dans le dictionnaire l'existance du charactère
                matrix = matrix + self.font5x3[text[i].upper()]   #code binaires correpondant à la lettre 
                matrix = matrix + [0]                             # ajout d'une colonne vide pour séparrer chaque lettre.
        return matrix


    #transmet un code binaire 8bit au convertisseur série->parallèle 74HC595
    #-----------------------------------------------------------------------
    def shiftOut_inv(self,val):
        #balayage gauche vers la droite par défaut (MSBFIRST)
        for i in range(8):
            GPIO.output(self.clockPin,GPIO.LOW); 
            GPIO.output(self.dataPin,(0x80&(val<<i)==0x80) and GPIO.HIGH or GPIO.LOW) #transmet 1 bit puis passe au suivant (val<<i)
            GPIO.output(self.clockPin,GPIO.HIGH); #le passsage HIGH value permet au 74HC595 d'acquérir un bit sur son port //

    def shiftOut(self,val):
        #balayage gauche vers la droite par défaut (MSBFIRST)
        for i in range(8):
            GPIO.output(self.clockPin,GPIO.LOW); 
            GPIO.output(self.dataPin,(0x1&(val>>i)==0x1) and GPIO.HIGH or GPIO.LOW) #transmet 1 bit puis passe au suivant (val<<i)
            GPIO.output(self.clockPin,GPIO.HIGH); #le passsage HIGH value permet au 74HC595 d'acquérir un bit sur son port //


    #affichage sur la matrice de leds depuis les 8 codes binaires fournis dans le buffer8b
    #-------------------------------------------------------------------------------------
    def affiche(self):
        x=0b10000000   # 1ère colonne
        for i in range(8): #codes binaires à transmettre, 1 pour chaque colonne
            GPIO.output(self.latchPin,GPIO.LOW) # prépare un changement des sorties parallèles
            if self.inverse:
                self.shiftOut_inv(self.buffer8bFinal[i]) # 8bits transmis au 1er étage 74HC595 = une barre verticale de 8 Leds
            else:
                self.shiftOut(self.buffer8bFinal[i])     # 8bits transmis au 1er étage 74HC595 = une barre verticale de 8 Leds
            self.shiftOut(~x)                   # 8 bits correspondant à la colonne à activier (LOW sur cette colonne, HIGH sur les autres, d'où ~x )
            GPIO.output(self.latchPin,GPIO.HIGH)  # sorties parralèles mises à jour sur les 2 étages
            x>>=1                                 # décallage 1 bit vers la droite = colonne suivante
            time.sleep(0.0015)                    # petit temps d'attente, compatible avec persistance rétinène

    #scroll un message texte sur la matrice de leds 
    #----------------------------------------------
    def scrollmsg(self, msg_txt):
        self.animation_squareF()
        self.afficheMsg=True
        msg_matrix = self.create_msg(msg_txt)   # transforme le texte en codes binaires
        for n in range(len(msg_matrix)-8):
            for i in range(8):
                self.buffer8b[i]=msg_matrix[n+i] << 1
            time.sleep(self.speedscroll)
        self.clear()
        self.afficheMsg=False


    #animation: carré qui s'Ouvre
    #-----------------------------------------
    def animation_squareO(self):
        anim_matrix = [    
            [0b00000000,0b00000000,0b00000000,0b00011000,0b00011000,0b00000000,0b00000000,0b00000000], 
            [0b00000000,0b00000000,0b00111100,0b00100100,0b00100100,0b00111100,0b00000000,0b00000000],
            [0b00000000,0b01111110,0b01000010,0b01000010,0b01000010,0b01000010,0b01111110,0b00000000],
            [0b11111111,0b10000001,0b10000001,0b10000001,0b10000001,0b10000001,0b10000001,0b11111111]
        ]
        self.animation(anim_matrix)

    #animation: carré qui se Ferme
    #--------------------------------------------
    def animation_squareF(self):
        anim_matrix = [    
            [0b11111111,0b10000001,0b10000001,0b10000001,0b10000001,0b10000001,0b10000001,0b11111111], 
            [0b00000000,0b01111110,0b01000010,0b01000010,0b01000010,0b01000010,0b01111110,0b00000000],
            [0b00000000,0b00000000,0b00111100,0b00100100,0b00100100,0b00111100,0b00000000,0b00000000],
            [0b00000000,0b00000000,0b00000000,0b00011000,0b00011000,0b00000000,0b00000000,0b00000000]
        ]      
        self.animation(anim_matrix)

    #affiche une animation en affichant les codes binaires fournis dans matrix 8 par 8
    #---------------------------------------------------------------------------------
    def animation(self, anim_matrix):
        self.afficheMsg=True
        for n in range(len(anim_matrix)):
            self.buffer8b = anim_matrix[n]
            time.sleep(self.speedscroll)
        self.clear()
        self.afficheMsg=False

    #affiche un logo fourni sous forme de liste de codes 8b, pendant t secondes
    #----------------------------------------------------------------------------
    def affiche_logo(self,logo8b):
        if len(logo8b)==8:
            self.buffer8b = logo8b

    #défini la valeur (vbin True|False) d'un pixel de la maitrice de LEds en pos(x,y)
    #(0,0) = coin bas gauche
    #--------------------------------------------------------------------------------
    def set_pixel(self, x,y, vbin):
        if vbin :
            self.buffer8b[x] = self.buffer8b[x] | self.puis2[y]    #allume le pixel
        else:
            self.buffer8b[x] = self.buffer8b[x] & ~self.puis2[y]   #éteint le pixel


        

