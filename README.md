# ESP32_1Wheel

Connections:
DIO 2 - Internal LED
DIO 16 - Safety switch input
DIO 17 - VESC output

Operation:
speed affected by pitch via PD controller
if pitching forward, speed up
if pitching back, slow down or reverse
if foot off safety switch, stop

Functionality not yet implemented:
if acceleration consistently upwards (ascending), increase pitch-speed ratio
if acceleration consistently downwards (descending), decrease pitch-speed ratio
if pitching back and going down a slope (ie fwdDir = true but pich < 0), brake (or reverse throttle)
if pitching forward x amount, accelerate to y speed. if pitch returned to ~0, continue at speed y
if pitching forward 2x amount, accelerate to 2y speed. if pitch returned to ~0, continue at speed y or 2y
if pitching back, decelerate from speed from y or 2y, deceleration proportional to pitch angle
