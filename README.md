# 🐱 HelloKitty – Projet Robot Chat
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
