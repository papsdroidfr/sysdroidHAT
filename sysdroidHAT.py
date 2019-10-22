#!/usr/bin/env python3
########################################################################
# Filename    : sysDroidHAT.py
# Description : Monitoring système raspberry pi (CPU, RAM, espace disque, T°, contrôle ventilateur)
# auther      : papsdroid.fr
# modification: 2019/10/05
########################################################################
import RPi.GPIO as GPIO
import time, os, threading, psutil
from stdLedMatrix import  LedMatrix

#classe affichage infos système (via thread)
#-----------------------------------------------------------------------------------------
class SysDroid(threading.Thread):
    def __init__(self, tFanMin, tFanMax, delay, speedscroll, inverse, verbose):     
        print ('Sysdroid démarre ... ')
        threading.Thread.__init__(self)  # appel au constructeur de la classe mère Thread

        self.etat=False             # état du thread False(non démarré), True (démarré)
        self.buttonSuivantPin = 40  # bouton pour changement d'affichage matrice de leds
        self.buttonOffPin = 38      # bouton d'extinction
        self.pic_id     = 0         # affichage sur la matrice de leds:  0(cpu, mem, T°), 1 (DISK)
        self.verbose = verbose      #True: active les print
        self.suivant= False         #True: bouton suivant activé

        #pin associés aux boutons poussoirs
        GPIO.setup(self.buttonSuivantPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)    # mode INPUT, pull_up=high
        GPIO.setup(self.buttonOffPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)        # mode INPUT, pull_up=high
        GPIO.add_event_detect(self.buttonSuivantPin,GPIO.FALLING,callback = self.buttonSuivantEvent, bouncetime=500)
        GPIO.add_event_detect(self.buttonOffPin,GPIO.FALLING,callback=self.buttonOffEvent, bouncetime=300)

        #affichage sur matrice de leds.
        self.ledmatrix = LedMatrix(speedscroll,inverse) #démarre l'affichage sur matrice de leds

        #animation de démarage sur la matrice de leds
        self.ledmatrix.animation_squareO()
        self.logo8b_1=[0b00001101, 0b00010101, 0b00010110, 0b00000000, 0b00000111, 0b00000010, 0b00000111,0]
        self.logo8b_2=[0b11001101, 0b10010101, 0b11010110, 0b10000000, 0b11000111, 0b10000010, 0b11000111,0]
        self.ledmatrix.affiche_logo(self.logo8b_1)
        for i in range(8): #effet ligne haut grandissante
            self.ledmatrix.set_pixel(i,7,True)
            time.sleep(0.05)
        for i in range(8): #effet ligne haut diminue
            self.ledmatrix.set_pixel(7-i,7,False)
            time.sleep(0.05)
        for i in range(8): #effet ligne haut grandissante
            self.ledmatrix.set_pixel(i,7,True)
            time.sleep(0.05)
        self.ledmatrix.affiche_logo(self.logo8b_2)
        time.sleep(1.5)

        # thread de lecture des informations système 
        self.readsys = ReadSys(tFanMin, tFanMax, delay, verbose)   
        self.readsys.start()                                       # démarrage du thread de lecture des info systèmes


    #fonction exécutée quand le bouton poussoir "OFF" est pressé
    #-----------------------------------------------------------
    def buttonOffEvent(self,channel):
        self.stop()
        print('Extinction Raspberry...')
        os.system('sudo halt')
    
        
    #fonction exécutée quand le bouton poussoir "suivant" est pressé
    #---------------------------------------------------------------
    def buttonSuivantEvent(self, channel):
        pic_id_next = (self.pic_id<1) * (self.pic_id + 1)
        #msg texte à afficher sur la matrice de leds
        if pic_id_next == 0:
            txt = " CPU:%.1f RAM:%.1f T°:%.1f°C " % (self.readsys.cpu_util, self.readsys.mem_used, self.readsys.cpu_t)
        else:
            txt = " DISK:%.1f " % (self.readsys.disk_used)
        self.ledmatrix.scrollmsg(txt)  #scrolle msg à l'écran
        self.pic_id = pic_id_next #provoque l'affichage nouvel écran quand le scrolling est terminé

        
    #exécution du thread
    #-------------------
    def run(self):
        self.etat=True
        while (self.etat):
            if (not(self.readsys.lu) and not(self.ledmatrix.afficheMsg)):
                self.ledmatrix.buffer8b = self.readsys.pic[self.pic_id]
                self.readsys.lu = True
            time.sleep(0.1) #attente pour ne pas saturer un proc
        
    #arrêt du thread
    #---------------
    def stop(self):
        self.etat=False
        self.readsys.stop()
        self.ledmatrix.stop()
        time.sleep(0.3) #attente pour que l'animation du thread LedMatrix soient bien terminée avant de libérer le GPIO
        GPIO.cleanup()
        print('Sysdroid arrêté')

#classe de lecture des informations systèmes à lire
#-----------------------------------------------------------------------------------------
class ReadSys(threading.Thread):
    def __init__(self, tFanMin, tFanMax, delay, verbose=False):
        threading.Thread.__init__(self)  # appel au constructeur de la classe mère Thread
        self.verbose = verbose      # True active les print
        self.etat=False                  # état du thread False(non démarré), True (démarré)
        self.delay = delay               # délay en secondes entre chaque nouvelle lecture
        self.t_min = 40                  # température minimale (0% si en dessous)
        self.t_max = 80                  # température maximale (100% si au dessus)
        self.fan_tOn  = tFanMax          # température d'activation du ventilateur
        self.fan_tOff = tFanMin          # température d'extinction du ventilateur
        self.cpu_t=0                     # température du CPU  
        self.cpu_util   = 0              # CPU global utilisation (%)
        self.cpus_util  = [0,0,0,0]      # CPUs utilisation (%)
        self.mem_used   = 0              # mémoire physique utilisée (%)
        self.disk_used  = 0              # usage du disk à la racine ('/') en %
        self.fanPin    = 8               # GPIO pin: control fan power
        GPIO.setup(self.fanPin, GPIO.OUT)
        GPIO.output(self.fanPin,GPIO.LOW) # fan off au début
        self.fanOn    = False             # True: ventilateur en marche, False: ventilateur à l'arrêt
        self.lu = False                   # True: données lues

        # codes binaires 8bits représentant les dessins (1 code par colonne) sur la matrice de leds
        #pic[0] représente les niveaux CPU (4), RAM,  T° CPU
        #pic[1] représente les niveaux espace disque à la racine de la carte SD ('/')
        self.pic = [ [0,0,0,0,0,0,0,0],     # pic[0] par defaut toutes les leds etteintes
                     [0,0,0,0,0,0,0,0]]     # pic[1] par defaut toutes les leds etteintes
        # correspondances des niveaux 0=0%, 1/8, 2/8,... 8/8=100% en codes binaires
        self.nivb = [0, 1, 0b11, 0b111, 0b1111, 0b11111, 0b111111, 0b1111111, 0b11111111] 

 
    #lecture de la température CPU
    #-----------------------------
    def get_cpu_temp(self):     
        tmp = open('/sys/class/thermal/thermal_zone0/temp')
        cpu = tmp.read()
        tmp.close()
        t=float(cpu)/1000
        if t<self.t_min:
            t=self.t_min
        if t>self.t_max:
            t=self.t_max
        return t 

    #converti la t° CPU en % entre t_min et t_max
    #--------------------------------------------
    def convert_cpu_pct(self):
        return (float)(self.cpu_t-self.t_min)/(self.t_max-self.t_min)*100

    #active ou désactive le ventilateur
    #----------------------------------
    def fan_chg(self, activation:True):
        GPIO.output(self.fanPin, activation and GPIO.HIGH or GPIO.LOW)
        self.fanOn = activation
        if self.verbose:
            print('Ventilateur', self.fanOn, 'cpu_t:',self.cpu_t,'°C')
        
    
    #démarrage du thread
    #-------------------
    def run(self):
        self.etat=True
        if self.verbose:
            print('Thread lecture info système démarré')
        while (self.etat):
            #lecture et stockage des informations système
            self.cpu_t = self.get_cpu_temp()
            self.cpu_util = psutil.cpu_percent()
            self.cpus_util = psutil.cpu_percent(percpu=True)
            self.mem_used = psutil.virtual_memory()[2]
            self.disk_used = psutil.disk_usage('/')[3]
            #self.disk_used = 85 #demo mode
            if self.verbose:
                print ('CPU:', self.cpu_util,'CPUs:', self.cpus_util,'% MEM used:',self.mem_used,'CPU T°:', self.cpu_t,'°C', ' DISK:',self.disk_used)
            #prépare les écrans à afficher sur la matrice de leds
            self.pic_levels() 
            #fan control
            if not(self.fanOn) and (self.cpu_t >= self.fan_tOn):
                self.fan_chg(True)
            elif self.fanOn and (self.cpu_t < self.fan_tOff): #extinction ventilateur
                self.fan_chg(False)
            self.lu = False         # données mises à jour à lire
            time.sleep(self.delay)  # mise en pause jusqu'à prochain rafraîchissement des données

    #arrêt du thread
    #---------------
    def stop(self):
        self.etat=False
        if self.verbose:
            print('Thread lecture info système stoppé')


    #prépare les écrans sur la matrice de leds
    #------------------------------------------------
    def pic_levels(self):

        #affichage 0: 4*CPU, MEM et T°
        #-------------------------------------------
        # 4 premieres colonnes: CPU utilisation
        for i in range(4):
            self.pic[0][i] = self.convert_level(self.cpus_util[i])
        # colonnes 5: utilisation mémoire
        self.pic[0][5] = self.convert_level(self.mem_used)
        # colonnes 6: température CPU
        self.pic[0][6] = self.convert_level(self.convert_cpu_pct())


        #affichage 1: disk usage. 1 led = 100/64 = 1.5625 % d'utilisation. 8 leds (1 colonne entière)=12.5%
        #-----------------------------------------------
        n = (int)(self.disk_used/12.5)          # nombre des barres verticales pleines 0 à 8
        r = (int)((self.disk_used%12.5)/1.5825) # différenciel avec barres pleines 0 à 7
        for i in range(n):
            self.pic[1][i] = 0xFF           # barres pleines = tranches d'utilisation à 12.5%
        if n<8:
            self.pic[1][n] = self.nivb[r]    # résiduel avec barres pleines
            for i in range(n+1,8):
                self.pic[1][i] = 0x00           # barres vides

    #converti un niveau de 0 à 100% en code binaire à 8 niveaux de 0b0 à 0b11111111
    #  correspondant à une barre de 8leds sur une colonne de la matrice de leds
    #  0 leds allumée: 00000000, 8 leds allumées: 11111111 etc...
    #-----------------------------------------------------------------------------
    def convert_level(self, niv):
        return ( self.nivb[round(niv/100*8+0.4999)] )       


#classe application principale
#------------------------------------------------------------------------------
class Application():
    def __init__(self,tFanMin=42, tFanMax=55, delay=1, speedscroll=0.1, inverse=False, verbose=False):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)  # localisation physique des GPIOs
        self.sysdroid = SysDroid(tFanMin, tFanMax, delay, speedscroll, inverse, verbose) 
        self.sysdroid.start()     # démarrage du thread de surveillance système
     	
    def loop(self):
        while True:
            time.sleep(1)

    def destroy(self):          # fonction exécutée sur appui CTRL-C
        self.sysdroid.stop()    # arrêt du thread de surveillance système


if __name__ == '__main__':     # Program start from here
    appl=Application(inverse=False)  
    try:
        appl.loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        appl.destroy()    
