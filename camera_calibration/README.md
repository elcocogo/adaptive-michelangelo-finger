# Positionnement des caméras stéréo

`live_view.py` : flux caméra live des deux `imx708`, côte à côte, pour
positionner physiquement les caméras — pas une calibration OpenCV (pas de
mire, rien n'est enregistré sur disque). Sert uniquement à vérifier que
les deux caméras gardent le bras dans le cadre sur toute son amplitude de
mouvement pendant que tu les orientes à la main.

`rpi502` n'a pas de bureau graphique, donc pas de fenêtre native possible.
Le flux est servi en MJPEG par HTTP à la place.

## Usage

```bash
cd ~/michelangelo && source .venv/bin/activate
python3 -m camera_calibration.live_view
```

Ouvre ensuite l'URL affichée dans un navigateur — `http://<ip-du-pi>:8100/`
(le port par défaut est 8100). Si tu es connecté via VS Code Remote-SSH,
VS Code propose en général de forwarder le port automatiquement dès qu'il
le détecte ouvert ; sinon utilise directement l'IP du Pi sur le réseau
local (le serveur écoute sur toutes les interfaces).

| Argument | Description |
|---|---|
| `--file` | Fichier de calibration à utiliser (défaut : `calibration_data/servos.json`) |
| `--port` | Port HTTP du flux MJPEG (défaut : 8100) |
| `--speed` | Vitesse de déplacement en % de la vitesse max supposée du servo, de 10 à 100 (défaut : 40) |

## Piloter le bras pendant le positionnement

Pendant que le flux tourne, le terminal reste disponible pour bouger les
deux premiers axes (les seuls utiles pour voir les positions limites du
bras) :

| Entrée | Action |
|---|---|
| `0 <angle\|c\|n\|x>` | canal 0 — rotation base / azimut |
| `1 <angle\|c\|n\|x>` | canal 1 — épaule (vertical à 0°, horizontal à ±90°) |
| `q` | quitte (arrête le flux et le serveur) |

`c`/`n`/`x` vont respectivement au centre, à la borne min et à la borne
max calibrées. Réutilise `move_to_angle` de `move_servo.py` : le
mouvement est donc rampé à `--speed`, et la position atteinte est
persistée dans `calibration_data/servo_positions.json` comme pour les
autres outils. Quitter avec `q` ne relâche pas le bras — même
comportement que `move_servo.py` et `arm_show.py`.

## Prérequis

- Les canaux 0 et 1 déjà calibrés (`calibrate_servo.py`).
- Rien à installer en plus : `picamera2` et `Pillow` viennent des paquets
  système déjà en place (`python3-picamera2`, voir `CLAUDE.md`), visibles
  dans le venv grâce à `--system-site-packages`.
