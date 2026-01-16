# 🐱 HelloKitty – Projet Robot Chat
<img width="810" height="1080" alt="image" src="https://github.com/user-attachments/assets/7b001b46-020f-4a15-b75d-87ee0aa59870" />


### **Les contributeurs :**
-BENJEMAA Aymen
-SOLTANI Ezer
-ROUABAH Serine

## 📚 Table des matières

1. [📜 Présentation générale](#-présentation-générale)
2. [📐 Architecture matérielle](#-architecture-matérielle)
   - [🔌 Schéma système](#-schéma-système)
   - [📌 Pinout STM32](#-pinout-stm32)
   - [🛠️ PCB et design électronique](#-pcb-et-design-électronique)
3. [🧩 Architecture logicielle](#-architecture-logicielle)
   - [🧱 Couches logicielles](#-couches-logicielles)
   - [🕒 Fonctionnement des tâches FreeRTOS](#-fonctionnement-des-tâches-freertos)
   - [🔄 Synchronisation et priorités](#-synchronisation-et-priorités)
4. [⚙️ Drivers et HAL](#-drivers-et-hal)
5. [🎯 Stratégie comportementale](#-stratégie-comportementale)
6. [📊 Tests et validation](#-tests-et-validation)
7. [🔧 Résultats et perspectives](#-résultats-et-perspectives)

## Présentation générale

HelloKitty est un robot mobile autonome conçu pour évoluer sur une surface plane sans bordure, dans un jeu de poursuite entre plusieurs robots. Le projet s’inscrit dans le cadre du module Systèmes Électroniques Avancés de l’ENSEA, et vise à couvrir l’ensemble du cycle de développement embarqué : de la conception du PCB à l’implémentation logicielle temps réel, en passant par la stratégie comportementale.

Le robot est capable de détecter les bords, d’éviter les chutes, de repérer d’autres robots, et de changer de rôle (chat ↔ souris) en fonction des interactions physiques ou visuelles. Le projet met en œuvre des capteurs variés, une architecture logicielle modulaire, et une gestion fine des tâches concurrentes via FreeRTOS.

## **Architecture**  
### **Schéma architectural**  

<img width="927" height="693" alt="schema_projet" src="https://github.com/user-attachments/assets/1b3bfaf2-3be3-44c0-845a-d9920e9071d3" />

#  Architecture du système embarqué STM32G431CBU6

Ce projet repose sur un microcontrôleur **STM32G431CBU6**, intégrant divers capteurs, modules de communication, moteurs et interfaces utilisateur pour créer un système autonome capable de capter, traiter et agir dans un environnement physique.

##  Microcontrôleur central
- **STM32G431CBU6** : cœur du système, gère les communications, le traitement des données et le contrôle des périphériques.

## Alimentation
- **Batterie NiMH 7.2V 1.3Ah** : source principale d’énergie.
- **Régulateurs de tension** :
  - **MP1475DJ-LF-P** : convertit la tension en **5V**.
  - **BU33SD5WG-TR** : convertit en **3.3V** pour les composants sensibles.

## Capteurs et modules
- **4 capteurs TOF (Time-of-Flight)** : connectés via **I2C**, pour mesurer les distances.
- **Accéléromètre ADXL343** : connecté en **I2C**, pour détecter les mouvements.
- **Module Bluetooth** : communication sans fil via **UART**.
- **Lidar YDLIDAR X2** : capteur de télémétrie, connecté en **UART**.

## Horloge et programmation
- **Quartz 16MHz** : fournit une horloge stable au microcontrôleur.
- **STLink SWD** : interface de programmation et débogage.
- **Hclk** : 170Mhz.

## Moteurs et contrôle
- **2 pilotes de moteur ZXBM5210** : reçoivent des signaux **PWM** pour contrôler les moteurs gauche et droit.
- **Moteurs avec encodeurs** : permettent un retour de position et de vitesse.

## 🖱️ Interface utilisateur
- **LED** : sortie **GPIO**, pour signalisation.
- **Bouton utilisateur** : entrée **GPIO**, pour interaction manuelle.
- **Bouton reset** : pour redémarrer le système.

## 🔌 Connexions colorées
- **Rouge** : lignes d’alimentation **5V**
- **Orange** : lignes **3.3V**
- **Violet** : **I2C**
- **Bleu** : **UART**
- **Vert** : **GPIO**
- **Noir** : **PWM**

---

Ce schéma illustre l’interconnexion des modules pour un système embarqué intelligent et réactif.

## Fonctionnement interne du robot

Le robot ne se contente pas d’exécuter des actions simples : son microcontrôleur coordonne en continu l’ensemble des capteurs, moteurs et modules pour produire un comportement cohérent et réactif. Cette section décrit la logique interne qui permet au système de fonctionner de manière autonome.

### Organisation logicielle
Le logiciel embarqué est structuré en plusieurs tâches indépendantes.  
Chaque tâche s’occupe d’un domaine précis : analyse des distances, lecture des chocs, gestion des moteurs ou encore surveillance de l’environnement.  
Cette organisation évite qu’une opération bloque les autres et garantit une réactivité constante.

### Système de décision
Le robot suit une hiérarchie de priorités pour réagir correctement aux événements :
- **Sécurité immédiate** : arrêt ou retrait en cas de danger (vide, obstacle trop proche, choc).
- **Évitement** : choix de la direction la plus dégagée grâce aux données du LiDAR.
- **Déplacement normal** : progression ou patrouille lorsque l’environnement est stable.

Cette logique empêche les comportements incohérents et permet des réactions rapides.

### Gestion dynamique des moteurs
Les moteurs sont ajustés en permanence selon la situation :
- correction de trajectoire,
- adaptation de la vitesse,
- compensation en cas de résistance ou de choc.

Le microcontrôleur calcule ces ajustements en temps réel, tandis que les drivers appliquent les consignes via PWM.

### Fusion des capteurs
Les informations issues des différents capteurs sont combinées pour obtenir une vision plus fiable de l’environnement :
- les ToF surveillent les bords,
- le LiDAR analyse l’espace autour du robot,
- l’accéléromètre détecte les impacts ou blocages.

Cette fusion permet d’anticiper les risques et d’adapter le comportement du robot de manière fluide.

### États internes
Le robot fonctionne comme une machine à états, chacun correspondant à un comportement précis :
- exploration,
- évitement,
- collision détectée,
- danger de chute,
- blocage,
- repos.

Chaque état définit les actions à effectuer et les conditions pour passer à un autre état.

### Indicateurs lumineux
Les LEDs servent de retour visuel pour comprendre l’état du robot :
- clignotement rapide : alerte,
- clignotement lent : attente,
- lumière fixe : fonctionnement normal.

Elles permettent de diagnostiquer rapidement le comportement du robot sans accéder au code.

### Synchronisation des communications
Les différents protocoles (UART, SPI, I2C) fonctionnent en parallèle.  
Pour éviter les conflits, les échanges sont cadencés et certaines lectures sont prioritaires.  
Les interruptions matérielles assurent la prise en charge immédiate des événements critiques.

---

Cette architecture logicielle permet au robot d’être autonome, réactif et capable de s’adapter en temps réel à son environnement.


##  Partie Hardware

Cette section décrit l’architecture matérielle du robot, ses composants électroniques, et les schémas associés.
<img width="983" height="564" alt="image" src="https://github.com/user-attachments/assets/481b72ed-1033-43fa-afc0-7f35f21c6596" />


### 🔌 Schéma global du système
Le système repose sur un microcontrôleur **STM32G431CBU6** qui coordonne les capteurs, les moteurs, les régulateurs et les interfaces utilisateur.


---

### ⚙️ Microcontrôleur et interfaces
Le microcontrôleur est au cœur du système. Il est connecté :
- aux moteurs via des signaux **PWM** et des entrées d’encodeurs,
- aux capteurs via **UART**, **I2C**, et **GPIO**,
- à un **STLink/SWD** pour la programmation et le débogage.

<img width="1078" height="742" alt="image" src="https://github.com/user-attachments/assets/20eed7e6-8ec8-43f0-a155-ee942acf7542" />


---

### 🔋 Alimentation et régulation
Le robot est alimenté par une batterie **NiMH 7.2V**, régulée en deux tensions :
- **5V** via le régulateur **MP1475DJ-LF-P**,
- **3.3V** via le régulateur **BU33SD5WG-TR**.

Ces tensions alimentent les moteurs, le microcontrôleur et les capteurs sensibles.

<img width="1013" height="592" alt="image" src="https://github.com/user-attachments/assets/fa8be56d-729b-44cd-a620-70aa3347fee2" />


---

### 🦾 Pilotes de moteurs
Chaque moteur est contrôlé par un circuit **ZXBM5210-SP**, avec :
- deux entrées **PWM** pour la vitesse et la direction,
- deux sorties vers le moteur (Motor+ / Motor−),
- des entrées d’encodeurs pour le retour de position.

Chaque moteur dispose de son propre driver et de ses propres signaux.

<img width="570" height="256" alt="image" src="https://github.com/user-attachments/assets/a448f5a5-3051-4683-b620-6901f1f0ada8" />

---

### 📡 Capteurs
Le système intègre :
- **4 capteurs TOF** pour la détection de bordure,
- **1 accéléromètre ADXL343** pour les chocs et mouvements,
- **1 LiDAR YDLIDAR X2** pour la cartographie et l’évitement,
- **1 module Bluetooth** pour la communication sans fil.

Tous ces capteurs sont connectés au microcontrôleur via **I2C**, **SPI**, **UART** ou **GPIO**.
<img width="1062" height="625" alt="image" src="https://github.com/user-attachments/assets/a634e98e-761b-432a-a299-bae5318e2d9d" />

---

### 🖱️ Interface utilisateur
Le robot dispose :
- de **LEDs** pour indiquer son état (obstacle, marche, pause…),
- d’un **bouton utilisateur** pour les interactions manuelles,
- d’un **bouton reset** pour redémarrer le système.
##im

---

### 🧩 Organisation des fichiers KiCad
Les schémas sont répartis en plusieurs fichiers :
- `pucontrolleur.kicad.sch` : microcontrôleur et interfaces
- `moteur1.kicad.sch` / `moteur2.kicad.sch` : circuits moteurs
- `regulateurs.kicad.sch` : alimentation
- `capteurs.kicad.sch` : capteurs et communication

---

Cette architecture matérielle permet au robot d’être autonome, réactif et modulaire. Chaque composant est interconnecté pour assurer un fonctionnement fluide et sécurisé.



https://github.com/user-attachments/assets/3a07851f-27b0-4f3c-a773-fe1d66b704f5


