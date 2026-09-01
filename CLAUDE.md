# Projet michelangelo — bras robotique + vision par ordinateur

## Objectif
Recréer le geste de la Création d'Adam de Michel-Ange : deux répliques
imprimées en 3D des doigts de la fresque se font face — l'une montée sur
un bras robotique 5 DOF, l'autre sur une tige tenue à la main. Le bras
suit la tige (repérée par vision stéréo, via un tag ArUco fixé dessus) et
pointe son propre doigt vers elle en temps réel, sans jamais la toucher.
Approche PBVS : reconstruction 3D par stéréovision, calibration
caméra→repère du bras, puis cinématique inverse.

Ce fichier (`CLAUDE.md`) est le journal de bord technique du projet, en
français. Le `README.md` à la racine est la documentation publique du
dépôt GitHub (en anglais, ne couvre que le hardware/software — la
conception mécanique et la construction physique sont documentées à
part, voir `3d_models/` et le dossier de notes de l'utilisateur, non
versionné).

## Matériel
- Raspberry Pi 5, nom d'hôte `rpi502`, dédié à ce projet (branché aux 2 caméras).
- 2x Raspberry Pi Camera Module v3, montées en paire stéréo fixe, calibrées
  (voir `camera_calibration/`).
- Contrôleur de servos PCA9685 (piloté en I2C directement depuis le Pi — pas d'Arduino,
  le PCA9685 génère le PWM en hardware, donc pas besoin d'un microcontrôleur dédié).
- Bras 5 DOF + PCA9685 câblés, 5 servos MG90S calibrés (voir `servo_calibration/`) :
  `base_joint` (azimut, Z) → `spin_joint` (épaule) → `basearm_joint` (coude)
  → `midarm_joint` (poignet) → `gripper_joint` (pince). Dimensions réelles
  mesurées : hauteur épaule 45mm, `base_arm` 54mm, `mid_arm` 50mm,
  `gripper_arm` 40mm (voir `kinematics/arm_kinematics.py`, `ArmDimensions`).
  Bornes calibrées (`calibration_data/servos.json`) : la plupart des axes sont
  à ±90°, mais `spin_joint` (épaule) a été recalibré en **-30°/+90°**
  (asymétrique) — ne pas supposer ±90° pour cet axe précis.
- Caméras câblées en CSI (ports 0 et 1), capteur imx708 (4608x2592).
- **PCA9685 câblé sur un bus I2C logiciel (`/dev/i2c-3`, GPIO23=SDA/GPIO24=SCL),
  pas sur le bus matériel standard.** Le bus matériel I2C1 (`/dev/i2c-1`,
  GPIO2/GPIO3, header pins 3/5) est tombé en panne le 2026-08-27 (erreurs kernel
  "lost arbitration", reproduites avec 3 cartes PCA9685 différentes → panne
  localisée au contrôleur I2C1/GPIO2-3 du Pi, pas aux cartes — probable
  survoltage pendant un rebranchement à chaud, l'alim servos étant en 5-6V
  contre 3.3V max toléré par les GPIO du Pi). Panne définitive, pas de
  réparation possible à ce niveau. Contournement :
  overlay `dtoverlay=i2c-gpio,bus=3` ajouté dans `/boot/firmware/config.txt`
  (bus I2C bit-bang sur GPIO23/24), confirmé fonctionnel et fiable depuis.
  Le driver (`servo_calibration/pca9685_driver.py`, `DEFAULT_I2C_BUS`) pointe
  désormais sur ce bus 3 par défaut. **Si jamais on rebranche un jour sur
  GPIO2/3 (I2C1 réparé ou Pi remplacé), remettre `DEFAULT_I2C_BUS = 1`**
  (ou passer `i2c_bus=1` explicitement) et retirer l'overlay du config.txt.
- Pièces mécaniques du bras dans `3d_models/` (`.FCStd` + `.step`) : design de
  base "grippy-bot" par ROBOTEURS (Cults3D, licence CC BY-NC — voir
  `3d_models/LICENSE.md`), retouché pièce par pièce dans FreeCAD pour les
  dimensions des servos MG90S.
- Deux répliques imprimées des doigts de la fresque : une fixée sur la pince
  du bras, une sur une tige tenue à la main (avec un tag ArUco pour le
  suivi par vision).
- Un second Raspberry Pi 5 existe (`rpi501`, Raspberry Pi OS Lite 64-bit) : relais
  pour uploader du code PlatformIO vers un Arduino Mega 2560, sur un autre projet.
  Il n'est PAS utilisé dans michelangelo.

## Décisions d'architecture (actées avec Claude, à respecter sauf changement explicite)
- **Tout tourne sur un seul Pi (`rpi502`)** : capture caméra, vision, cinématique
  inverse, pilotage PCA9685 — pas de répartition réseau entre les deux Pi.
- **Python pur** (pas de ROS2).
- **OS : Ubuntu Server 26.04 LTS (64-bit)**, choisi volontairement (plutôt que
  Raspberry Pi OS) pour préparer une intégration ROS2 ultérieure : la distribution
  ROS2 LTS actuelle, "Lyrical Luth" (sortie mai 2026), cible justement Ubuntu 26.04 LTS
  — donc installation ROS2 par apt le moment venu, sans souci de compatibilité.
- Pas d'interface graphique (Server, pas Desktop) : travail en SSH. Depuis le
  2026-08-26, le développement se fait **directement sur `rpi502`** via
  VS Code Remote-SSH avec l'extension Claude Code installée côté Pi.
- **ChArUco plutôt qu'un damier classique** pour la calibration stéréo : reste
  détectable même partiellement hors-cadre ou occulté, contrairement à un
  damier qui doit être vu en entier.
- **Calibration caméra→bras par 2 tags ArUco au sol** (positions mesurées à la
  main + normale au sol lue directement sur les tags) plutôt qu'une hand-eye
  calibration classique (bouger le bras à travers des poses connues) : cette
  dernière aurait nécessité une cinématique directe déjà fonctionnelle, ce qui
  n'était pas encore le cas à ce stade du projet.
- **Redondance cinématique (4 axes pour une cible 3D)** gérée par deux solveurs
  distincts selon le besoin : `inverse_kinematics_search` (cherche un angle de
  poignet qui garde tous les axes dans leurs bornes calibrées) et
  `inverse_kinematics_pointing` (solution fermée qui garde en plus le dernier
  segment aligné avec la cible). Voir `kinematics/README.md`.
- **Pas de vérification visuelle du bout du bras** (pas de second tag ArUco
  dessus) : la cinématique en boucle ouverte s'est avérée assez précise en
  pratique, ajout jugé non nécessaire.
- Point d'attention : le support caméra (`picamera2`/`libcamera`) est moins
  turnkey sur Ubuntu que sur Raspberry Pi OS — installs manuels explicites
  (`libcamera`, `rpicam-apps`, `python3-picamera2` via apt).

## Feuille de route
1. ✅ OS + configuration système
2. ✅ Câblage physique (PCA9685 sur bus I2C logiciel GPIO23/24 suite à la panne
   I2C1, voir Matériel) + alim servos séparée + montage stéréo des caméras
3. ✅ Bring-up logiciel — caméras (`camera_calibration/live_view.py`), PCA9685
   détecté (bus 3), 5 servos calibrés, bras pilotable par angle
   (`servo_calibration/move_servo.py`), chorégraphie de démo
   (`servo_calibration/arm_show.py`), retour position initiale
   (`servo_calibration/go_home.py`)
4. ✅ Calibration stéréo (`camera_calibration/calibrate_stereo.py`) — résultat :
   ~0.2px RMS par caméra, ~0.4px pour la paire stéréo
5. ✅ Calibration caméra → repère du bras
   (`camera_calibration/calibrate_camera_to_arm.py`) — résultat : 1.8% d'erreur
   sur la distance mesurée vs triangulée, 3.2° d'accord entre les deux normales
6. ✅ Cinématique directe/inverse du bras (`kinematics/arm_kinematics.py`) —
   validée par des allers-retours FK→IK→FK randomisés, erreur sub-micron
7. ✅ Boucle de suivi (`tracking/follow_target.py`) — détection cible →
   triangulation 3D → repère bras → IK → PCA9685, fonctionnelle en continu,
   avec aperçu live et enregistrement vidéo en option

Le pipeline hardware/software de bout en bout est fonctionnel. Pistes non
retenues pour l'instant (voir Décisions d'architecture) : vérification
visuelle du bout du bras, détection de doigt à mains nues (le concept final
retient un tag ArUco sur la tige plutôt qu'une détection de main).

## Statut actuel
`rpi502` est opérationnel et accessible en SSH depuis le Mac via l'alias `ssh rpi502`
(config dans `~/.ssh/config` sur le Mac, IP DHCP — pas d'IP fixe pour l'instant).
Note : `rpi502.local` (mDNS) ne résout pas — `avahi-daemon` n'est pas installé par
défaut sur Ubuntu Server. Pas bloquant, l'alias SSH compense.

Paquets installés sur rpi502 : `i2c-tools`, `rpicam-apps`, `python3-picamera2`.
L'I2C du GPIO header est actif et accessible sans sudo (utilisateur `cgo` dans
le groupe `i2c`) — le PCA9685 est câblé sur le bus logiciel `/dev/i2c-3`
(GPIO23/24) suite à la panne du bus matériel I2C1 (voir Matériel). Vérifier sa
détection avec `i2cdetect -y 3` (adresse par défaut du PCA9685 : 0x40).

Dépôt Git : poussé sur GitHub, `github.com/elcocogo/adaptive-michelangelo-finger`
(public), branche `main`. `README.md` à la racine documente l'ensemble du
projet (hardware + pipeline logiciel + résultats + marche à suivre).

## Packages logiciels

### `servo_calibration/`
Calibration et pilotage des servos. `calibrate_servo.py` (calibration
interactive au clavier, pulse centre + bornes min/max, résultats dans
`calibration_data/servos.json` — non versionné), `move_servo.py` (pilotage
par angle avec rampe de vitesse, persistance de la position entre
invocations), `arm_show.py` (chorégraphie de démo, mouvements synchronisés
multi-axes), `go_home.py` (retour à la position verticale, tous axes à 0°).
Détail complet dans `servo_calibration/README.md`.

### `camera_calibration/`
`live_view.py` (aperçu live MJPEG des 2 caméras pour positionnement
physique, avec pilotage des 2 premiers axes du bras depuis le terminal),
`generate_targets.py` (génère les cibles à imprimer — mire ChArUco 7x5 +
tags ArUco individuels ; taille nominale 25mm, taille réellement mesurée
après impression 25.6mm, voir `charuco_board.py::MEASURED_SQUARE_LENGTH_MM`),
`capture_stereo_images.py` + `calibrate_stereo.py` (calibration stéréo),
`triangulation.py` (2D+2D → 3D générique), `aruco_markers.py` (détection de
tags individuels), `calibrate_camera_to_arm.py` (calibration caméra→repère
du bras). Détail complet dans `camera_calibration/README.md`.

### `kinematics/`
`arm_kinematics.py` — géométrie pure, aucune dépendance hardware.
`forward_kinematics`/`inverse_kinematics` (chaîne réduite à un bras plan +
rotation de base, grâce aux axes parallèles des 3 articulations
intermédiaires), `inverse_kinematics_search` (résout la redondance en
cherchant un angle de poignet qui respecte les bornes calibrées),
`inverse_kinematics_pointing` (solution fermée qui garde en plus le dernier
segment pointé exactement vers la cible), `apply_standoff` (recul le long
de la droite épaule→cible). Détail complet dans `kinematics/README.md`.

### `tracking/`
`follow_target.py` — boucle complète de suivi visuel, assemble tous les
packages ci-dessus. Options : `--point-gripper` (aligne le dernier segment
sur la cible), `--standoff-mm` (distance de sécurité, défaut 135mm),
`--record` (enregistrement vidéo de l'aperçu annoté), `--layout`. Retour
à la position initiale (`go_home`) à la sortie (`q` ou Ctrl+C). Détail
complet dans `tracking/README.md`.
