# ROS 2 Interface

Namespace:

```
/<RobotName>/<Port>/
```

Verfügbare Sensoren:

## Compass sensor

### Publisher

Type: Float32
Liefer Werte im Bereich 0 bis 2pi
0 ist Nordpol, im Uhrzeigersinn aufsteigende Werte
Topic: `compass`

## Distance sensor

### Publisher

Type: Int16
Topic: `distance`

## Light sensor

### Publisher

Type: Int16
Liefer Werte im Bereich 1600 bis 3200. Je kleiner der Wert, desto heller das Licht
Topic: `light`

## Touch sensor

### Publisher

Type: Bool
`True` wenn der Sensor gedrückt wird, ansonsten `False`
Topic: `touch`

## Motor

### Subscriptions

**Set velocity**

Type: Float32
Topic: `setVelocity`

Setzt neue Geschwindigkeit des Motors. Wird gecapped auf _[VelocityLimit.MIN, VelocityLimit.MAX]_

**Set power**

Type: Int16
Topic: `setPower`

Setzt neue Power des Motors. Wird gecapped auf _[PowerLimit.MIN, PowerLimit.MAX]_

**Set position**

Type: Float64
Topic: `setPosition`

Setze neue Position des Motors. Wird in _rad_ angegeben.

**Set position offset**

Type: Float64
Topic: `setPositionOffset`

Setze den Positionsoffset des Motors. Wird in _rad_ angegeben.
Bsp:
Die aktuelle Position des Motors beträgt _0rad_. Nun wird der
Offset auf _PI_ gesetzt. Auslesen der Position des Motors würde nun das Ergebnis _-PI_ bringen.

**Reset position offset**

Type: Empty
Topic: `resetPositionOffset`

Setzt den Positionsoffset genau auf die aktuelle Position des Motors. Die neue Position des Motors wäre nach auslesen dann immer _0_.

**Set velocity limit**

Type: Float32MultiArray -----> TODO ändern in nur Float32
Topic: `setVelocityLimit`

Setzt die Werte _[VelocityLimit.MIN, VelocityLimit.MAX]_.

**Set power limit**

Type: Int16MultiArray -----> TODO ändern in nur Int16
Topic: `setPowerLimit`

Setzt die Werte _[PowerLimit.MIN, PowerLimit.MAX]_.

### Publisher

**Position**

Type: Float64
Topic: `position`

**Velocity**

Type: Float32
Topic: `velocity`

**Power**

Type: Int16
Topic: `power`

**Flags**

Type: Int16
Topic: `flags` ---> REMOVE

**Power limit**

Type: Int16MultiArray
Topic: `powerLimit`

**Velocity limit**

Type: Float32MultiArray
Topic: `velocityLimit`
