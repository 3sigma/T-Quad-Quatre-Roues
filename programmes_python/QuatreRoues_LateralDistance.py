#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de contrôle du robot T-Quad avec 4 roues holonomes (roues Mecanum)
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Suivi de mur
#
# Auteur: 3Sigma
# Version 1.1.1 - 30/01/2017
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino import *

# Imports Généraux
import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Gestion de l'écran OLED
from oled.device import ssd1306
from oled.render import canvas
from PIL import ImageFont, ImageDraw

# Gestion de l'IMU
from mpu9250 import MPU9250


# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Mega
from mega import Mega
mega = Mega(hostname = hostname)

# Moteurs
Nmoy = 1

omegaArriereDroit = 0.
codeurArriereDroitDeltaPos = 0
codeurArriereDroitDeltaPosPrec = 0

omegaArriereGauche = 0.
codeurArriereGaucheDeltaPos = 0
codeurArriereGaucheDeltaPosPrec = 0

omegaAvantDroit = 0.
codeurAvantDroitDeltaPos = 0
codeurAvantDroitDeltaPosPrec = 0

omegaAvantGauche = 0.
codeurAvantGaucheDeltaPos = 0
codeurAvantGaucheDeltaPosPrec = 0

# Tension effectivement appliquée
commandeArriereDroit = 0.
commandeArriereGauche = 0.
commandeAvantDroit = 0.
commandeAvantGauche = 0.

# Saturations
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Asservissement
Kplat = 3.3 # gain proportionnel du PID d'asservissement latéral
Kilat = 82.0 # gain intégral du PID d'asservissement latéral
Kdlat = 0. # gain dérivé du PID d'asservissement latéral
Tflat = 0.02 # constante de temps de filtrage de l'action dérivée du PID d'asservissement latéral
Kprot = 0.25 # gain proportionnel du PID d'asservissement de rotation
Kirot = 10.2 # gain intégral du PID d'asservissement de rotation
Kdrot = 0. # gain dérivé du PID d'asservissement de rotation
Tfrot = 0.02 # constante de temps de filtrage de l'action dérivée du PID d'asservissement de rotation
Kpdist = 10. # gain proportionnel du PID d'asservissement de distance
Kidist = 2.5 # gain intégral du PID d'asservissement de distance
Kddist = 2.0 # gain dérivé du PID d'asservissement de distance
Tfdist = 0.5 # constante de temps de filtrage de l'action dérivée du PID d'asservissement de distance
I_x = [0., 0., 0., 0.]
D_x = [0., 0., 0., 0.]
yprec = [0., 0., 0., 0.] # mesure de la vitesse du moteur droit au calcul précédent
vxmes = 0.
vymes = 0.
ximes = 0.
commandeLongi = 0.
commandeLat = 0.
commandeRot = 0.

# Paramètres mécaniques
R = 0.0225 # Rayon d'une roue
W = 0.18 # Ecart entre le centre de rotation du robot et les roues

# Variables utilisées pour les données reçues ou fixées
distref = 20.
vyref = 0.
xiref = 0.
rapport_xigz = 0.5

# Timeout de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.05
i = 0
tprec = time.time()
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

idecimLectureTension = 0
decimLectureTension = 6000
decimErreurLectureTension = 100

# Mesure de la tension de la batterie
# On la contraint à être supérieure à 7V, pour éviter une division par
# zéro en cas de problème quelconque
lectureTensionOK = False
tensionAlim = 7.4
while not lectureTensionOK:
    try:
        tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
        lectureTensionOK = True
    except:
        print("Erreur lecture tension")

# Capteur de distance
pulse_start = 0
pulse_end = 0
pulse_duration = 0
last_pulse_duration = 0
distance = 0
distancePrec = 0
distanceFiltre = 0
tauFiltreDistance = 0.1

if (hostname == "pcduino"):
    trig = 10
    echo = 13
    # Initialisation
    pinMode(trig, OUTPUT)
    pinMode(echo, INPUT)
elif (hostname == "raspberrypi"):
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    trig = 3 # GPIO22
    echo = 23
    # Initialisation
    pinMode(trig, OUTPUT)
    GPIO.setup(echo,GPIO.IN)
else:
    # pcDuino par défaut
    trig = 10
    echo = 13
    # Initialisation
    pinMode(trig, OUTPUT)
    pinMode(echo, INPUT)

# Initialisation de l'IMU
gz = 0.
if (hostname == "pcduino"):
    I2CBUS = 2
elif (hostname == "raspberrypi"):
    I2CBUS = 1
else:
    # pcDuino par défaut
    I2CBUS = 2
    
initIMU_OK = False
while not initIMU_OK:
    try:
        imu = MPU9250(i2cbus=I2CBUS, address=0x69)
        initIMU_OK = True
    except:
        print("Erreur init IMU")


#--- setup --- 
def setup():
    
    # Initialisation des moteurs
    CommandeMoteurs(0, 0, 0, 0)
    
    # Initialisation du capteur de distance
    digitalWrite(trig, LOW)
    print "Attente du capteur de distance"
    time.sleep(2)
    
    digitalWrite(trig, HIGH)
    time.sleep(0.00001)
    digitalWrite(trig, LOW)

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --

def CalculVitesse():
    global omegaArriereDroit, omegaArriereGauche, omegaAvantDroit, omegaAvantGauche, timeLastReceived, timeout, timedOut, \
        tdebut, codeurArriereDroitDeltaPos, codeurArriereGaucheDeltaPos, codeurAvantDroitDeltaPos, codeurAvantGaucheDeltaPos, \
        commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche, \
        codeurArriereDroitDeltaPosPrec, codeurArriereGaucheDeltaPosPrec, codeurAvantDroitDeltaPosPrec, codeurAvantGaucheDeltaPosPrec, tprec, \
        idecimLectureTension, decimLectureTension, decimErreurLectureTension, tensionAlim, commandeLongi, commandeLat, commandeRot, \
        pulse_start, pulse_end, pulse_duration, last_pulse_duration, distance, distancePrec, \
        distanceFiltre, tauFiltreDistance, imu, gz, R, W, vxmes, vymes, ximes, distref, vyref, xiref, font, rapport_xigz, \
        Kpdist, Kidist, Kddist, Tfdist, hostname
    
    tdebut = time.time()
        
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        codeursDeltaPos = mega.read_codeursDeltaPos()
        codeurArriereDroitDeltaPos = codeursDeltaPos[0]
        codeurArriereGaucheDeltaPos = codeursDeltaPos[1]
        codeurAvantDroitDeltaPos = codeursDeltaPos[2]
        codeurAvantGaucheDeltaPos = codeursDeltaPos[3]
        
        # Suppression de mesures aberrantes
        if (abs(codeurArriereDroitDeltaPos - codeurArriereDroitDeltaPosPrec) > 10) or (abs(codeurArriereGaucheDeltaPos - codeurArriereGaucheDeltaPosPrec) > 10) or (abs(codeurAvantDroitDeltaPos - codeurAvantDroitDeltaPosPrec) > 10) or (abs(codeurAvantGaucheDeltaPos - codeurAvantGaucheDeltaPosPrec) > 10):
            codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
            codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
            codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
            codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec


        codeurArriereDroitDeltaPosPrec = codeurArriereDroitDeltaPos
        codeurArriereGaucheDeltaPosPrec = codeurArriereGaucheDeltaPos
        codeurAvantDroitDeltaPosPrec = codeurAvantDroitDeltaPos
        codeurAvantGaucheDeltaPosPrec = codeurAvantGaucheDeltaPos
    except:
        #print "Error getting data"
        codeurArriereDroitDeltaPos = codeurArriereDroitDeltaPosPrec
        codeurArriereGaucheDeltaPos = codeurArriereGaucheDeltaPosPrec
        codeurAvantDroitDeltaPos = codeurAvantDroitDeltaPosPrec
        codeurAvantGaucheDeltaPos = codeurAvantGaucheDeltaPosPrec
    
    # A la fin de l'expression, on divise par 0.01 car c'est à cette cadence que le calcul des delta codeurs
    # est effectué sur l'Arduino
    omegaArriereDroit = -2 * ((2 * 3.141592 * codeurArriereDroitDeltaPos) / 1200) / (Nmoy * 0.01)  # en rad/s
    omegaArriereGauche = 2 * ((2 * 3.141592 * codeurArriereGaucheDeltaPos) / 1200) / (Nmoy * 0.01)  # en rad/s
    omegaAvantDroit = -2 * ((2 * 3.141592 * codeurAvantDroitDeltaPos) / 1200) / (Nmoy * 0.01)  # en rad/s
    omegaAvantGauche = 2 * ((2 * 3.141592 * codeurAvantGaucheDeltaPos) / 1200) / (Nmoy * 0.01)  # en rad/s
    
    # Mesures
    vxmes = (omegaArriereDroit + omegaArriereGauche + omegaAvantDroit + omegaAvantGauche) * R / 4
    vymes = (omegaArriereDroit - omegaArriereGauche - omegaAvantDroit + omegaAvantGauche) * R / 4
    ximes = (omegaArriereDroit - omegaArriereGauche + omegaAvantDroit - omegaAvantGauche) * R / W / 4
    
    # Lecture de la vitesse de rotation autour de la verticale
    try:
        gyro = imu.readGyro()
        gz = gyro['z'] * math.pi / 180
    except:
        #print("Erreur lecture IMU")
        pass

    dt2 = time.time() - tprec
    tprec = time.time()
    
    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout and not timedOut:
        timedOut = True
        
    if timedOut:
        commandeLongi = 0.
        commandeLat = 0.
        commandeRot = 0.
    else:
        commandeLongi = PID(0, -distref/100., -distanceFiltre/100., Kpdist, Kidist, Kddist, Tfdist, umax, umin, dt2);
        commandeLat = PID(1, vyref, vymes, Kplat, Kilat, Kdlat, Tflat, umax, umin, dt2);
        # On mélange la mesure par odométrie et par le gyro
        commandeRot = PID(2, xiref, ximes * (rapport_xigz) + gz * (1. - rapport_xigz), Kprot, Kirot, Kdrot, Tfrot, umax, umin, dt2);
    
    # Transformation des commandes longitudinales et de rotation en tension moteurs
    commandeArriereDroit = -(commandeLongi + commandeLat + commandeRot) # Tension négative pour faire tourner positivement ce moteur
    commandeArriereGauche = commandeLongi - commandeLat - commandeRot
    commandeAvantDroit = -(commandeLongi - commandeLat + commandeRot) # Tension négative pour faire tourner positivement ce moteur
    commandeAvantGauche = commandeLongi + commandeLat - commandeRot
    
    CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche)
    
    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlim = max(7.0, float(mega.read_battery_millivolts()) / 1000.)
            idecimLectureTension = 0
        except:
            # On recommence la lecture dans decimErreurLectureTension * dt
            idecimLectureTension = idecimLectureTension - decimErreurLectureTension
            #print("Erreur lecture tension dans Loop")
    else:
        idecimLectureTension = idecimLectureTension + 1
    
    # Calcul de la distance mesurée par le capteur ultrason
    # On fait ce calcul après l'affichage pour savoir combien de temps
    # il reste pour ne pas perturber la boucle
    digitalWrite(trig, HIGH)
    time.sleep(0.00001)
    digitalWrite(trig, LOW)
    
    if (hostname == "pcduino"):
        pulse_duration = 0
        while (digitalRead(echo) == 0) and (time.time() - tdebut < dt):
            pulse_start = time.time()

        while (digitalRead(echo) == 1) and (pulse_duration < 0.01166) and (time.time() - tdebut < dt):
            pulse_end = time.time()
            last_pulse_duration = pulse_duration
            pulse_duration = pulse_end - pulse_start
            
    elif (hostname == "raspberrypi"):
        pulse_duration = 0
        while (GPIO.input(echo) == 0) and (time.time() - tdebut < dt):
            pulse_start = time.time()

        while (GPIO.input(echo) == 1) and (pulse_duration < 0.01166) and (time.time() - tdebut < dt):
            pulse_end = time.time()
            last_pulse_duration = pulse_duration
            pulse_duration = pulse_end - pulse_start
            
    else:
        pulse_duration = 0
        while (digitalRead(echo) == 0) and (time.time() - tdebut < dt):
            pulse_start = time.time()

        while (digitalRead(echo) == 1) and (pulse_duration < 0.01166) and (time.time() - tdebut < dt):
            pulse_end = time.time()
            last_pulse_duration = pulse_duration
            pulse_duration = pulse_end - pulse_start
                
    distance = last_pulse_duration * 17150
    #distance = round(distance, 0)
    if (distance == 0):
        # C'est probablement une mesure aberrante, on la supprime
        distance = distancePrec
    # Filtre sur la distance
    distanceFiltre = (dt2 * distance + tauFiltreDistance * distancePrec) / (dt2 + tauFiltreDistance)
    distancePrec = distanceFiltre

        
    #print time.time() - tdebut

    
def PID(iMoteur, omegaref, omega, Kp, Ki, Kd, Tf, umax, umin, dt2):
    global I_x, D_x, yprec
    
    # Calcul du PID
    # Paramètres intermédiaires
    Ti = Ki/(Kp+0.01)
    if (Kd>0): # Si PID
        ad = Tf/(Tf+dt2)
        bd = Kd/(Tf+dt2)
        Td = Kp/Kd
        Tt = sqrt(Ti*Td)
    else: # Si PI
        ad = 0
        bd = 0
        Td = 0
        Tt = 0.5*Ti
    
    br = dt2/(Tt+0.01)

    # Calcul de la commande avant saturation
        
    # Terme proportionnel
    P_x = Kp * (omegaref - omega)

    # Terme dérivé
    D_x[iMoteur] = ad * D_x[iMoteur] - bd * (omega - yprec[iMoteur])

    # Calcul de la commande avant saturation
    commande_avant_sat = P_x + I_x[iMoteur] + D_x[iMoteur]

    # Application de la saturation sur la commande
    if (commande_avant_sat > umax):
        commande = umax
    elif (commande_avant_sat < umin):
        commande = umin
    else:
        commande = commande_avant_sat
    
    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_x[iMoteur] = I_x[iMoteur] + Ki * dt2 * (omegaref - omega) + br * (commande - commande_avant_sat)
    
    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprec[iMoteur] = omega
    
    return commande


def CommandeMoteurs(commandeArriereDroit, commandeArriereGauche, commandeAvantDroit, commandeAvantGauche):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    global tensionAlim
    
    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionArriereDroit = commandeArriereDroit
    tensionArriereGauche = commandeArriereGauche
    tensionAvantDroit = commandeAvantDroit
    tensionAvantGauche = commandeAvantGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_ArriereDroit = int(255 * tensionArriereDroit / tensionAlim)
    tension_int_ArriereGauche = int(255 * tensionArriereGauche / tensionAlim)
    tension_int_AvantDroit = int(255 * tensionAvantDroit / tensionAlim)
    tension_int_AvantGauche = int(255 * tensionAvantGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_ArriereDroit > 255):
        tension_int_ArriereDroit = 255

    if (tension_int_ArriereDroit < -255):
        tension_int_ArriereDroit = -255

    if (tension_int_ArriereGauche > 255):
        tension_int_ArriereGauche = 255

    if (tension_int_ArriereGauche < -255):
        tension_int_ArriereGauche = -255

    if (tension_int_AvantDroit > 255):
        tension_int_AvantDroit = 255

    if (tension_int_AvantDroit < -255):
        tension_int_AvantDroit = -255

    if (tension_int_AvantGauche > 255):
        tension_int_AvantGauche = 255

    if (tension_int_AvantGauche < -255):
        tension_int_AvantGauche = -255

    # Commande PWM
    try:
        mega.moteursArriere(tension_int_ArriereDroit, tension_int_ArriereGauche)
        mega.moteursAvant(tension_int_AvantDroit, tension_int_AvantGauche)
        mega.moteursCRC(tension_int_ArriereDroit + tension_int_ArriereGauche, tension_int_AvantDroit + tension_int_AvantGauche)
    except:
        pass
        #print "Erreur moteurs"

    
def emitData():
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    #delay(5000)
    tprec = time.time()
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 50)
        self.callback.start()
    

    def on_message(self, message):
        global distref, vyref, rapport_xigz, Kpdist, Kidist, Kddist, distanceFiltre, tauFiltreDistance, timeLastReceived, timedOut
            
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
        
        if jsonMessage.get('distref') != None:
            distref = float(jsonMessage.get('distref'))
        if jsonMessage.get('vyref') != None:
            vyref = float(jsonMessage.get('vyref')) / 100.
        if jsonMessage.get('rapport_xigz') != None:
            # Choix de la source de la vitesse de rotation mesurée: 0: 100% gyro, 1: 100% vitesse des roues
            rapport_xigz = float(jsonMessage.get('rapport_xigz'))
        if jsonMessage.get('Kp') != None:
            Kpdist = float(jsonMessage.get('Kp'))
        if jsonMessage.get('Ki') != None:
            Kidist = float(jsonMessage.get('Ki'))
        if jsonMessage.get('Kd') != None:
            Kddist = float(jsonMessage.get('Kd'))
        if jsonMessage.get('tauFiltreDistance') != None:
            tauFiltreDistance = float(jsonMessage.get('tauFiltreDistance'))
                
        if not socketOK:
            vyref = 0.
  

    def on_close(self):
        global socketOK, vxref, vyref, xiref
        print 'connection closed...'
        socketOK = False
        vxref = 0.
        vyref = 0.
        xiref = 0.

    def sendToSocket(self):
        global socketOK, vxmes, vymes, ximes, omegaArriereDroit, omegaArriereGauche, omegaAvantDroit, omegaAvantGauche, \
            distance, distanceFiltre, gz, distref, vyref, commandeLongi, commandeLat
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), \
                                'consigne_distance':("%d" % distref), \
                                'consigne_vy':("%.2f" % (100 * vyref)), \
                                'vxmes':("%.2f" % vxmes), \
                                'vymes':("%.2f" % (100 * vymes)), \
                                'ximes':("%.2f" % ximes), \
                                'distance':("%d" % distance), \
                                'distanceFiltre':("%d" % distanceFiltre), \
                                'commandeLongi':("%.2f" % commandeLongi), \
                                'commandeLat':("%.2f" % commandeLat), \
                                'gz':("%.2f" % gz), \
                                'Raw':("%.2f" % tcourant) \
                                + "," + ("%.2f" % vxmes) \
                                + "," + ("%.2f" % (100 * vymes)) \
                                + "," + ("%.2f" % ximes) \
                                + "," + ("%d" % distance) \
                                + "," + ("%d" % distanceFiltre) \
                                + "," + ("%.2f" % commandeLongi) \
                                + "," + ("%.2f" % commandeLat) \
                                + "," + ("%.2f" % gz) \
                                })
                                
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vxref, vyref, xiref
    print 'Sortie du programme'
    vxref = 0.
    vyref = 0.
    xiref = 0.
    CommandeMoteurs(0, 0, 0, 0)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
    setup() # appelle la fonction setup
    print "Setup done."
    
    th = threading.Thread(None, emitData, None, (), {})
    th.daemon = True
    th.start()
    
    print "Starting Tornado."
    try:
        print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
    except:
        pass
        
    try:
        print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
    except:
        pass
    socketOK = False
    startTornado()


