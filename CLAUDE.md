# Projet michelangelo — bras robotique + vision par ordinateur

## Objectif
Bras robotique articulé, avec suivi d'objet par vision (asservissement visuel,
approche PBVS : reconstruction 3D par stéréovision puis cinématique inverse).

## Matériel
- Raspberry Pi 5, nom d'hôte `rpi502`, dédié à ce projet (branché aux 2 caméras).
- 2x Raspberry Pi Camera Module v3, montées en paire stéréo (baseline fixe à définir).
- Contrôleur de servos PCA9685 (piloté en I2C directement depuis le Pi — pas d'Arduino,
  le PCA9685 génère le PWM en hardware, donc pas besoin d'un microcontrôleur dédié).
- Bras + PCA9685 reçus, câblage pas encore fait (prochaine étape physique).
- Caméras déjà câblées en CSI (ports 0 et 1) et validées fonctionnelles (capture
  réussie sur les deux, capteur détecté : imx708, 4608x2592). PCA9685 pas encore
  câblé sur le bus I2C GPIO (`/dev/i2c-1`, vide pour l'instant, normal).
- Pince ("gripper") en cours de conception sous FreeCAD, dossier `grippy-bot/`
  (versions successives : V02_Base, V02_BaseArm, V02_BaseSpin, V02_Finger,
  V02_GripperArm, V02_MidArm — fichiers .FCStd/.step/.STL).
- Un second Raspberry Pi 5 existe (`rpi501`, Raspberry Pi OS Lite 64-bit) : relais
  pour uploader du code PlatformIO vers un Arduino Mega 2560, sur un autre projet.
  Il n'est PAS utilisé dans michelangelo.

## Décisions d'architecture (actées avec Claude, à respecter sauf changement explicite)
- **Tout tourne sur un seul Pi (`rpi502`)** : capture caméra, vision, cinématique
  inverse, pilotage PCA9685 — pas de répartition réseau entre les deux Pi.
- **Python pur pour démarrer** (pas de ROS2 dans un premier temps).
- **OS : Ubuntu Server 26.04 LTS (64-bit)**, choisi volontairement (plutôt que
  Raspberry Pi OS) pour préparer une intégration ROS2 ultérieure : la distribution
  ROS2 LTS actuelle, "Lyrical Luth" (sortie mai 2026), cible justement Ubuntu 26.04 LTS
  — donc installation ROS2 par apt le moment venu, sans souci de compatibilité.
- Pas d'interface graphique (Server, pas Desktop) : travail en SSH, comme pour `rpi501`.
  Depuis le 2026-08-26, le développement se fait **directement sur `rpi502`** via
  VS Code Remote-SSH avec l'extension Claude Code installée côté Pi (plus besoin de
  faire un aller-retour Mac → rsync → Pi comme au tout début du projet).
- Point d'attention pour plus tard : le support caméra (`picamera2`/`libcamera`) est
  moins turnkey sur Ubuntu que sur Raspberry Pi OS — nécessite des installs manuels
  explicites (`libcamera`, `rpicam-apps`, `python3-picamera2` via apt), documentés
  mais pas préinstallés.

## Feuille de route
1. ✅ OS + configuration système (Ubuntu Server 26.04 flashé, SSH + WiFi configurés,
   connexion SSH validée depuis le Mac)
2. Câblage physique (PCA9685 sur I2C + alim servos séparée) — caméras déjà faites,
   reste le PCA9685 et le montage stéréo définitif (baseline à fixer)
3. Bring-up logiciel isolé — caméras : ✅ fait (capture testée sur les 2, cam0 et cam1).
   I2C GPIO : ✅ activé et accessible (`i2c-tools` installé, bus `/dev/i2c-1` OK,
   vide en attendant le PCA9685). Reste : détecter le PCA9685 une fois câblé, faire
   bouger un servo.
4. Calibration stéréo (intrinsèques/extrinsèques, mire échiquier, cv2.stereoCalibrate)
5. Calibration caméra → repère du bras
6. Cinématique inverse du bras (à écrire selon le nombre d'axes/segments réels)
7. Boucle de suivi (détection cible → triangulation 3D → repère bras → IK → PCA9685)

## Statut actuel
`rpi502` est opérationnel et accessible en SSH depuis le Mac via l'alias `ssh rpi502`
(config dans `~/.ssh/config` sur le Mac, pointant vers son IP DHCP actuelle
192.168.1.124 — pas d'IP fixe pour l'instant, à revoir si l'IP change trop souvent).
Note : `rpi502.local` (mDNS) ne résout pas — `avahi-daemon` n'est pas installé par
défaut sur Ubuntu Server (contrairement à Raspberry Pi OS). Pas bloquant, l'alias SSH
compense, mais à installer si besoin de confort plus tard.

Paquets installés sur rpi502 : `i2c-tools`, `rpicam-apps`, `python3-picamera2`.
Les deux caméras (imx708) sont détectées et capturent correctement. L'I2C du GPIO
header est actif et accessible sans sudo (utilisateur `cgo` dans le groupe `i2c`).

Prochaine étape physique : câbler le PCA9685 sur le bus I2C (GPIO), avec une
alimentation séparée pour les servos (ne pas alimenter les servos depuis le 5V du Pi),
puis vérifier sa détection avec `i2cdetect -y 1` (adresse par défaut du PCA9685 : 0x40).

## Outil de calibration des servos
`servo_calibration/` (package Python, code et usage documentés dans le docstring de
`calibrate_servo.py`) : outil interactif au clavier pour trouver par tâtonnement,
servo par servo, le pulse centre (0°) et les pulses min/max correspondant aux angles
limites choisis à l'oeil. Les résultats se cumulent dans `calibration_data/servos.json`
(un calibrage ne touche que l'entrée de son propre canal PCA9685). Dépendances dans
`requirements.txt` — venv déjà créé sur rpi502 dans `~/michelangelo/.venv` avec
`--system-site-packages` (nécessaire pour `python3-lgpio`, voir commentaire en tête
de `requirements.txt`). Usage :
```
cd ~/michelangelo && source .venv/bin/activate
python3 -m servo_calibration.calibrate_servo --channel 0 --name shoulder
```
Pas encore testé avec un vrai PCA9685 branché (câblage pas fait au moment de l'écriture).
