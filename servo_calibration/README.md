# Calibration et pilotage des servos (PCA9685)

Trois outils en ligne de commande pour travailler avec les servos branchés
sur le PCA9685 :

- **`calibrate_servo.py`** : à lancer une fois par servo, juste après
  l'avoir monté sur le bras, pour trouver à la main son impulsion centre
  (0°) et ses bornes min/max.
- **`move_servo.py`** : à lancer ensuite, autant de fois que besoin, pour
  piloter un servo déjà calibré en lui donnant directement un angle en
  degrés.
- **`arm_show.py`** : une fois les 5 servos calibrés, une petite
  chorégraphie de démonstration qui enchaîne des poses sur le bras
  complet.

Les trois s'appuient sur le même driver ([pca9685_driver.py](pca9685_driver.py))
et la même calibration partagée ([calibration.py](calibration.py)), stockée
dans `calibration_data/servos.json` à la racine du projet.

## Prérequis

- PCA9685 câblé (I2C sur le header GPIO + alim externe séparée sur le bloc
  V+) et détecté :
  ```bash
  i2cdetect -y 1   # doit montrer 40 à la ligne 40
  ```
- Environnement virtuel activé :
  ```bash
  cd ~/michelangelo && source .venv/bin/activate
  ```

## Fichier de calibration

`calibration_data/servos.json` est un unique fichier JSON, partagé par tous
les servos, indexé par numéro de canal PCA9685 :

```json
{
  "pwm_frequency_hz": 50,
  "servos": {
    "0": {
      "channel": 0,
      "name": "shoulder",
      "pulse_min_us": 900.0,
      "pulse_center_us": 1500.0,
      "pulse_max_us": 2100.0,
      "angle_min_deg": -90.0,
      "angle_max_deg": 90.0,
      "calibrated_at": "2026-08-26T12:00:00+00:00"
    }
  }
}
```

Chaque calibration (sauvegarde) ne touche que l'entrée de son propre canal :
calibrer ou recalibrer un servo ne risque jamais d'écraser les autres.

`move_servo.py` tient par ailleurs à jour un second fichier,
`calibration_data/servo_positions.json`, qui retient le dernier angle
*commandé* par canal (voir la section dédiée plus bas) — pas besoin d'y
toucher à la main, il est géré automatiquement.

## 1. `calibrate_servo.py` — calibration interactive

À faire une fois par servo (ou pour recalibrer un servo déjà monté).

```bash
python3 -m servo_calibration.calibrate_servo --channel 0 --name shoulder
```

| Argument | Description |
|---|---|
| `--channel` | Canal PCA9685 du servo (0-15), obligatoire |
| `--name` | Nom du servo (ex. `shoulder`, `elbow`). Optionnel si le canal a déjà une calibration existante — reprend son nom |
| `--file` | Fichier de calibration à utiliser (défaut : `calibration_data/servos.json`) |
| `--frequency` | Fréquence PWM en Hz (défaut : 50) |

**Contrôles au clavier (touche seule, sans Entrée) :**

| Touche | Action |
|---|---|
| `h` / `l` | diminue / augmente l'impulsion d'un pas (10µs par défaut) |
| `H` / `L` | idem, pas x10 |
| `[` / `]` | divise / double la taille du pas |
| `c` | marque la position actuelle comme le centre (0°) |
| `n` | marque la position actuelle comme borne min (demande l'angle, ex. `-90`) |
| `x` | marque la position actuelle comme borne max (demande l'angle, ex. `90`) |
| `p` | affiche l'état courant |
| `r` | coupe le PWM (relâche le servo) |
| `s` | sauvegarde dans le fichier de calibration |
| `?` | réaffiche l'aide |
| `q` | quitte (demande confirmation si non sauvegardé) |

**Procédure, à chaque nouveau servo branché :**

1. Brancher le servo sur un canal libre du PCA9685, noter le canal et
   l'articulation qu'il représente.
2. Lancer l'outil avec ce canal et un nom explicite.
3. Amener doucement le servo (`h`/`l`, `H`/`L`, ajuster le pas avec
   `[`/`]`) à la position voulue pour le 0°, puis `c`.
4. Continuer jusqu'à la limite mécanique choisie dans un sens — **en
   s'arrêtant avant la butée dure**, jamais dessus — puis `n` et l'angle
   correspondant.
5. Même chose dans l'autre sens, puis `x`.
6. Vérifier avec `p` que les 3 pulses et les 2 angles sont cohérents.
7. Sauvegarder avec `s`.
8. `r` pour relâcher le servo (utile pour manipuler le bras à la main
   ensuite), puis `q` pour quitter.

Quoi qu'il arrive, l'impulsion envoyée reste bornée à
`[400, 2600]` µs (`HARD_PULSE_MIN_US`/`HARD_PULSE_MAX_US` dans
`calibrate_servo.py`) — une protection logicielle qui ne remplace pas la
vigilance sur les butées mécaniques réelles pendant la calibration.

## 2. `move_servo.py` — pilotage par angle

Une fois un servo calibré, pour le positionner directement à un angle
donné (ex. après un redémarrage, pour tester une pose, ou remettre un
servo à 0° avant de continuer le montage du bras).

```bash
python3 -m servo_calibration.move_servo --channel 0
# ou
python3 -m servo_calibration.move_servo --name shoulder
# ou, sans argument, il demande le canal ou le nom au lancement
python3 -m servo_calibration.move_servo
```

| Argument | Description |
|---|---|
| `--channel` | Canal PCA9685 du servo à piloter |
| `--name` | Nom du servo (tel qu'enregistré lors de la calibration) |
| `--file` | Fichier de calibration à utiliser (défaut : `calibration_data/servos.json`) |
| `--speed` | Vitesse de déplacement en % de la vitesse max supposée du servo, de 10 à 100 (défaut : 70) |

`--channel` et `--name` sont mutuellement exclusifs ; si aucun des deux
n'est fourni, l'outil le demande de façon interactive au démarrage.

**Limitation de vitesse (`--speed`)** : le PCA9685 ne fait qu'imposer une
largeur d'impulsion, il n'a aucun contrôle natif sur la vitesse du servo —
un servo commandé d'un coup de min à max accélère donc au maximum de ses
capacités, ce qui peut être violent pour un bras encore fragile. `move_servo.py`
compense en envoyant des positions intermédiaires (rampe), à 50 mises à
jour par seconde, entre l'angle courant et l'angle demandé. `--speed 100`
correspond à une vitesse pleine échelle supposée (`MAX_SERVO_SPEED_DEG_PER_S`
dans le script, 300°/s par défaut — une estimation générique pour un servo
hobby, à ajuster si besoin), `--speed 10` déplace 10x plus lentement.

Comme le pilotage est en boucle ouverte (pas de retour de position réel),
la rampe a besoin de connaître l'angle de départ. Le PCA9685 continue de
piloter un canal à sa dernière impulsion commandée même après la fermeture
du script — donc `move_servo.py` retient le dernier angle commandé dans
`calibration_data/servo_positions.json` et le recharge au lancement
suivant. Résultat : `--speed` s'applique dès la toute première commande
d'une nouvelle invocation, tant que le servo n'a pas été relâché (`r`) ou
déplacé à la main entre-temps — ce qui est le cas d'usage le plus courant
(une commande, puis on quitte).

Quitter avec `q` **ne coupe pas le PWM** : le servo garde sa position.
Seul `r` relâche le servo, et efface aussi la position enregistrée
puisqu'elle n'est plus fiable après (le bras peut avoir bougé à la main).

**Contrôles :**

| Entrée | Action |
|---|---|
| un nombre (ex. `12.5`, `-30`) | déplace le servo à cet angle en degrés |
| `c` | va au centre (0°) |
| `n` | va à la borne min calibrée |
| `x` | va à la borne max calibrée |
| `r` | coupe le PWM (relâche le servo), oublie la position enregistrée |
| `q` | quitte — le servo garde sa position (PWM toujours actif) |

Un angle en dehors de `[angle_min_deg, angle_max_deg]` est **refusé avec un
message d'erreur**, sans bouger le servo :

```
> 200
  Erreur : 200.0 deg hors bornes [-90.0, 90.0] pour 'shoulder'.
```

Le script ne déplace jamais le servo automatiquement au démarrage — la
première commande est toujours explicite.

## 3. `arm_show.py` — chorégraphie de démonstration

Une fois les 5 canaux (0 à 4) calibrés, enchaîne automatiquement une série
de poses sur le bras complet : extension au loin, repli compact, rotation
de la base sur 180°, pliage du coude et du poignet, ouverture/fermeture de
la pince, puis quelques mouvements combinés (plusieurs articulations à la
fois).

```bash
python3 -m servo_calibration.arm_show
# vitesse et pause entre poses ajustables :
python3 -m servo_calibration.arm_show --speed 40 --pause 1.0
```

| Argument | Description |
|---|---|
| `--file` | Fichier de calibration à utiliser (défaut : `calibration_data/servos.json`) |
| `--speed` | Vitesse de déplacement en % de la vitesse max supposée du servo, de 10 à 100 (défaut : 60) |
| `--pause` | Pause en secondes entre deux poses (défaut : 0.6) |

Chaque pose ne cite que les canaux qu'elle change ; les autres restent où
ils étaient. Quand une pose bouge plusieurs canaux à la fois, le
mouvement est **synchronisé** : le canal qui doit parcourir le plus grand
angle fixe la durée du déplacement (à `--speed`), les autres canaux de
cette même pose sont interpolés sur la même durée pour arriver tous
ensemble, plutôt que de finir en escalier.

Les canaux 1/2/3 (`base_arm`/`mid_arm`/`gripper_arm`) suivent une
convention d'angle *relative au segment parent* (voir la calibration
initiale) : `mid_arm=0°` ne veut pas dire qu'il pointe vers le haut, mais
qu'il continue tout droit dans la direction de `base_arm`, quelle qu'elle
soit.

Comme `move_servo.py`, le script réutilise `calibration_data/servo_positions.json`
pour connaître la position réelle au démarrage, et ne relâche jamais le
bras automatiquement (`Ctrl+C` interrompt proprement le show sans laisser
le bras dans un état incohérent — il garde sa dernière pose).

## Sécurité

- Alimentation des servos (bloc V+ du PCA9685) toujours séparée du 5V du
  Pi, sur une alim externe dédiée.
- Impulsion toujours bornée en dur côté logiciel (`calibrate_servo.py`),
  mais ça ne dispense pas de rester attentif aux butées mécaniques réelles
  du bras pendant une calibration.
- `r` (relâcher / couper le PWM) est disponible dans les deux outils —
  à utiliser avant toute manipulation à la main du bras.
- Dans `move_servo.py` et `arm_show.py`, quitter (`q`, fin normale, ou
  `Ctrl+C`) **ne relâche pas** les servos : ils restent activement
  maintenus à leur dernière position commandée. C'est voulu (ces outils
  servent justement à fixer une position et la garder), mais ça veut dire
  qu'un servo peut rester sous tension/couple après la fin
  du script — pense à `r` si tu veux le relâcher explicitement.
